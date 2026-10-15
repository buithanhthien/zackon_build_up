import ssl
import tempfile
import unittest
from email.message import Message
from io import BytesIO
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import certifi
from robot_ui.web_fact_cache import WebFactCache, collect_web_facts, fetch_source
from robot_ui.conversation_policy import answer_turn


class WebFactCacheTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.now = 1000
        self.cache = WebFactCache(Path(self.tmp.name) / 'web.sqlite3',
                                  ttls={'room': 10}, clock=lambda: self.now)
        self.query = 'Phòng cô Đặng Thanh Tâm ở IUH?'
        self.url = 'https://iuh.edu.vn/teacher'
        self.quote = 'Năm 2022, Đặng Thanh Tâm tại IUH: phòng X5.1-4.'
        self.fact = dict(subject='Đặng Thanh Tâm', attribute='room', value='X5.1-4',
                         scope='IUH', kind='room', quote=self.quote, url=self.url,
                         source_date='2022')
        self.pages = {self.url: dict(title='IUH', text=self.quote)}

    def save(self):
        return self.cache.store(self.query, [self.fact], self.pages, complete=True)

    def turn(self, question=None, memory=None, external=False):
        self.web = Mock(return_value='live')
        self.compose = Mock(return_value='Theo bạn, X5.2.5; trang năm 2022 nêu X5.1-4.')
        return answer_turn(
            dict(route='local', query=self.query, external_required=external),
            question or self.query, '',
            search_local=Mock(return_value=SimpleNamespace(status='insufficient')),
            answer_local=Mock(), answer_web=self.web, answer_general=Mock(),
            answer_memory=Mock(return_value=memory), search_cache=self.cache.lookup,
            answer_cached=self.compose)

    def test_repeat_uses_durable_cache_then_expiry_calls_web(self):
        self.turn()
        self.web.assert_called_once()
        self.assertTrue(self.save())
        self.turn()
        self.web.assert_not_called()
        self.assertEqual(WebFactCache(self.cache.path, clock=lambda: self.now).lookup(self.query)[0]['provenance'], 'web')
        self.now += 10
        self.turn()
        self.web.assert_called_once()

    def test_correction_and_source_time_reach_single_composer(self):
        self.save()
        self.turn(memory='Theo bạn, phòng X5.2.5')
        self.web.assert_not_called()
        args = self.compose.call_args.args
        self.assertEqual(args[2][0]['source_date'], '2022')
        self.assertIn('X5.2.5', args[3])

    def test_explicit_lookup_always_bypasses_cache_and_memory(self):
        self.save()
        for question in ('Tìm trên mạng', 'Kiểm tra hiện tại trên trang trường',
                         'Kiểm tra trên trang trường'):
            self.turn(question, memory='X5.2.5')
            self.web.assert_called_once()
            self.compose.assert_not_called()
        self.turn(external=True)
        self.web.assert_called_once()

    def test_no_cross_identity_or_scope_reuse(self):
        self.save()
        self.assertEqual(self.cache.lookup('Phòng cô Đặng Thanh Tâm ở trường khác?'), [])
        self.fact['scope'] = 'Trường khác'
        self.assertFalse(self.save())

    def test_rejects_model_prose_unbacked_values_and_dates(self):
        self.assertFalse(self.cache.store(self.query, [self.fact], {}, complete=True))
        self.assertTrue(self.cache.store(self.query, [self.fact], self.pages))
        self.fact['value'] = 'X5.2.5'
        self.assertFalse(self.save())
        self.fact['value'] = 'X5.1-4'
        self.fact['source_date'] = '2026'
        self.assertFalse(self.save())

    def test_extractor_only_receives_source_text(self):
        import json
        client = Mock()
        client.chat.completions.create.return_value = SimpleNamespace(choices=[
            SimpleNamespace(message=SimpleNamespace(content=json.dumps(
                {'complete': True, 'facts': [self.fact]})))])
        fetch = Mock(return_value=self.quote)
        self.assertTrue(collect_web_facts(client, 'test', self.query,
            [{'url': self.url, 'title': 'IUH'}], self.cache, fetch=fetch))
        fetch.assert_called_once_with(self.url)
        self.assertIn(self.quote, client.chat.completions.create.call_args.kwargs['messages'][1]['content'])

    def test_fetch_source_uses_verified_certifi_context(self):
        headers = Message()
        headers['Content-Type'] = 'text/plain; charset=utf-8'
        response = Mock()
        response.__enter__ = Mock(return_value=response)
        response.__exit__ = Mock(return_value=False)
        response.headers = headers
        response.read.return_value = b'Official source'
        opener = Mock()
        opener.open.return_value = response
        with patch('robot_ui.web_fact_cache.socket.getaddrinfo', return_value=[
                (None, None, None, None, ('8.8.8.8', 443))]), \
             patch('robot_ui.web_fact_cache.ssl.create_default_context',
                   wraps=ssl.create_default_context) as create_context, \
             patch('robot_ui.web_fact_cache.build_opener', return_value=opener) as build_opener_mock:
            self.assertEqual(fetch_source('https://iuh.edu.vn/source'), 'Official source')
        create_context.assert_called_once()
        self.assertEqual(create_context.call_args.kwargs['cafile'],
                         certifi.where())
        self.assertEqual(len(build_opener_mock.call_args.args), 2)
        context = build_opener_mock.call_args.args[1]._context
        self.assertEqual(context.verify_mode, ssl.CERT_REQUIRED)
        self.assertTrue(context.check_hostname)

    def test_fetch_source_extracts_text_from_pdf(self):
        from pypdf import PdfWriter

        pdf = BytesIO()
        writer = PdfWriter()
        writer.add_blank_page(width=72, height=72)
        writer.write(pdf)
        headers = Message()
        headers['Content-Type'] = 'application/pdf'
        response = Mock()
        response.__enter__ = Mock(return_value=response)
        response.__exit__ = Mock(return_value=False)
        response.headers = headers
        response.read.return_value = pdf.getvalue()
        opener = Mock()
        opener.open.return_value = response
        with patch('robot_ui.web_fact_cache.socket.getaddrinfo', return_value=[
                (None, None, None, None, ('8.8.8.8', 443))]), \
             patch('robot_ui.web_fact_cache.build_opener', return_value=opener):
            with self.assertRaisesRegex(ValueError, 'no extractable text'):
                fetch_source('https://iuh.edu.vn/source.pdf')

    def test_fetch_source_extracts_pdf_text(self):
        from pypdf import PdfWriter
        from pypdf.generic import DecodedStreamObject, DictionaryObject, NameObject

        pdf = BytesIO()
        writer = PdfWriter()
        page = writer.add_blank_page(width=72, height=72)
        font = DictionaryObject({
            NameObject('/Type'): NameObject('/Font'),
            NameObject('/Subtype'): NameObject('/Type1'),
            NameObject('/BaseFont'): NameObject('/Helvetica'),
        })
        font_reference = writer._add_object(font)
        page[NameObject('/Resources')] = DictionaryObject({
            NameObject('/Font'): DictionaryObject({NameObject('/F1'): font_reference}),
        })
        content = DecodedStreamObject()
        content.set_data(b'BT /F1 12 Tf 10 10 Td (Official IUH source) Tj ET')
        page[NameObject('/Contents')] = writer._add_object(content)
        writer.write(pdf)
        headers = Message()
        headers['Content-Type'] = 'application/octet-stream'
        response = Mock()
        response.__enter__ = Mock(return_value=response)
        response.__exit__ = Mock(return_value=False)
        response.headers = headers
        response.read.return_value = pdf.getvalue()
        opener = Mock()
        opener.open.return_value = response
        with patch('robot_ui.web_fact_cache.socket.getaddrinfo', return_value=[
                (None, None, None, None, ('8.8.8.8', 443))]), \
             patch('robot_ui.web_fact_cache.build_opener', return_value=opener):
            self.assertEqual(fetch_source('https://iuh.edu.vn/source.pdf'),
                             'Official IUH source')

    def test_partial_answer_keeps_valid_fact_and_logs_rejections(self):
        bad = {**self.fact, 'value': 'X5.2.5'}
        with self.assertLogs('robot_ui.web_fact_cache', level='INFO') as logs:
            self.assertTrue(self.cache.store(self.query, [bad, self.fact, None], self.pages))
        self.assertEqual(len(self.cache.lookup(self.query)), 1)
        self.assertIn('extracted=3 saved=1', ' '.join(logs.output))
        self.assertIn('unsupported_value_or_identity', ' '.join(logs.output))

    def test_paraphrase_preserves_scope_and_identity(self):
        self.save()
        self.assertTrue(self.cache.lookup('Cho tôi biết phòng làm việc của cô Đặng Thanh Tâm tại IUH ở đâu?'))
        self.assertFalse(self.cache.lookup('Phòng cô Đặng Tâm Thanh ở IUH?'))
        self.assertFalse(self.cache.lookup('Phòng cô Đặng Thanh Tâm ở IUH khoa khác?'))

    def test_each_fact_expires_independently_and_conflicts_remain(self):
        second = {**self.fact, 'value': 'X5.2.5', 'quote': self.quote + ' X5.2.5', 'kind': 'other'}
        self.pages[self.url]['text'] = second['quote']
        self.cache.store(self.query, [self.fact, second], self.pages)
        self.assertEqual(len(self.cache.lookup(self.query)), 2)
        self.now += 10
        self.assertEqual([f['value'] for f in self.cache.lookup(self.query)], ['X5.2.5'])
        self.assertEqual(self.cache.inspect()['facts'], 2)
        self.assertEqual(self.cache.inspect()['unexpired'], 1)

    def test_same_name_ambiguous_scopes_miss(self):
        self.save()
        other = {**self.fact, 'scope': 'Trường khác', 'quote': self.quote + ' Trường khác'}
        self.pages[self.url]['text'] = other['quote']
        self.cache.store(self.query, [other], self.pages)
        self.assertEqual(self.cache.lookup('Đặng Thanh Tâm là ai?'), [])
        self.assertEqual([f['scope'] for f in self.cache.lookup(self.query)], ['IUH'])

    def test_write_error_logs_specific_reason(self):
        with patch.object(self.cache, '_connect', side_effect=OSError('disk full')):
            with self.assertLogs('robot_ui.web_fact_cache', level='ERROR') as logs:
                with self.assertRaises(OSError):
                    self.save()
        self.assertIn('disk full', ' '.join(logs.output))

    def test_explicit_web_request_overrides_general_route(self):
        web = Mock(return_value='live')
        answer = answer_turn(
            dict(route='general', query=self.query, external_required=False),
            'Tìm trên web thông tin cô Đặng Thanh Tâm', '',
            search_local=Mock(), answer_local=Mock(), answer_web=web,
            answer_general=Mock(), answer_memory=Mock(return_value=None),
            search_cache=self.cache.lookup, answer_cached=Mock())
        self.assertEqual(answer, 'live')
        web.assert_called_once()

    def save_role(self, subject='Đặng Thanh Tâm', scope='IUH'):
        quote = f'{subject} tại {scope}: Giáo vụ'
        fact = dict(subject=subject, scope=scope, attribute='chức danh',
                    value='Giáo vụ', kind='personnel', quote=quote, url=self.url)
        self.cache.store('Tra cứu nhân sự', [fact],
                         {self.url: dict(title='Trang khoa', text=quote)})

    def test_role_recall_across_questions_and_restart(self):
        self.save_role()
        cache = WebFactCache(self.cache.path, clock=lambda: self.now)
        for question in ('cô Tâm giáo vụ là ai?', 'cô Đặng Thanh Tâm làm gì?',
                         'cho biết thêm về cô Đặng Thanh Tâm',
                         'Cho mình biết thêm thông tin về cô Đặng Thanh Tâm',
                         'Đặng Thanh Tâm'):
            with self.subTest(question=question):
                facts = cache.lookup(question)
                self.assertEqual([f['value'] for f in facts], ['Giáo vụ'])
        self.assertEqual(cache.lookup('Viết truyện về cô Đặng Thanh Tâm'), [])
        self.assertEqual(cache.lookup('Dịch câu cô Đặng Thanh Tâm là giáo vụ'), [])
        self.assertEqual(cache.lookup('cô Tâm là ai?'), [])

    def test_short_name_ambiguity_and_scope(self):
        self.save_role()
        self.save_role('Nguyễn Thanh Tâm', 'Trường B')
        self.assertEqual(self.cache.lookup('cô Tâm giáo vụ là ai?'), [])
        facts = self.cache.lookup('cô Tâm giáo vụ tại IUH là ai?')
        self.assertEqual([f['subject'] for f in facts], ['Đặng Thanh Tâm'])
        self.assertEqual(self.cache.lookup('Đặng Thanh Tâm tại Trường B là ai?'), [])

    def test_general_route_composes_partial_evidence_with_memory_and_local(self):
        self.save_role()
        question = 'Cho mình biết thêm thông tin về cô Đặng Thanh Tâm'
        local = SimpleNamespace(status='sufficient', evidence=['repository'])
        compose = Mock(return_value='Trang khoa ghi cô là giáo vụ. Chưa có thêm thông tin.')
        web = Mock()
        general = Mock(return_value='normal')
        memory = Mock(return_value='Theo bạn, cô làm tại phòng A')
        kwargs = dict(search_local=Mock(return_value=local), answer_local=Mock(),
                      answer_web=web, answer_general=general, answer_memory=memory,
                      search_cache=self.cache.lookup, answer_cached=compose)
        answer = answer_turn(dict(route='general', query=question, external_required=False),
                             question, '', **kwargs)
        self.assertIn('Chưa có thêm thông tin', answer)
        args = compose.call_args.args
        self.assertEqual(args[2][0]['value'], 'Giáo vụ')
        self.assertEqual(args[3], 'Theo bạn, cô làm tại phòng A')
        self.assertIs(args[4], local)
        web.assert_not_called()
        general.assert_not_called()
        self.assertEqual(answer_turn(dict(route='general', query='Xin chào', external_required=False),
                                     'Xin chào', '', **kwargs), 'normal')

    def test_lookup_diagnostics_explain_skips(self):
        self.save_role()
        with self.assertLogs('robot_ui.web_fact_cache', level='INFO') as logs:
            self.cache.lookup('Đặng Thanh Tâm tại trường khác')
        self.assertIn('scope_or_qualifier_mismatch', ' '.join(logs.output))
        self.assertIn('hits=0', ' '.join(logs.output))
