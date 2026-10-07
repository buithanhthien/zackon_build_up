"""Strict Vietnamese motion grammar and transport-independent validation."""
import math
import re
import unicodedata

MAX_ACTIONS = 12
LIMITS = {'revolutions': 5, 'degrees': 1800, 'duration_seconds': 30, 'distance_meters': 5}


def normalize(text):
    text = unicodedata.normalize('NFD', text.lower().replace('đ', 'd'))
    text = ''.join(c for c in text if unicodedata.category(c) != 'Mn')
    return re.sub(r'\s+', ' ', text).strip().rstrip('.!')


def number(text):
    if re.fullmatch(r'\d+(?:[.,]\d+)?', text):
        return float(text.replace(',', '.'))
    digits = dict(zip('khong mot hai ba bon nam sau bay tam chin'.split(), range(10)))
    digits.update(tu=4, lam=5)
    tokens = text.split()
    if not tokens:
        raise ValueError('Thiếu số lượng.')
    if len(tokens) == 1 and tokens[0] in digits:
        return digits[tokens[0]]
    decimal_separator = next((word for word in ('phay', 'cham') if word in tokens), None)
    if decimal_separator:
        left, right = text.split(f' {decimal_separator} ', 1)
        if not right.split() or any(t not in digits for t in right.split()):
            raise ValueError('Số thập phân không hợp lệ.')
        return number(left) + float('0.' + ''.join(str(digits[t]) for t in right.split()))
    total = 0
    if len(tokens) >= 2 and tokens[0] in digits and tokens[1] == 'tram':
        total = digits[tokens.pop(0)] * 100
        tokens.pop(0)
        if tokens and tokens[0] in ('linh', 'le'):
            tokens.pop(0)
    if tokens and tokens[0] == 'muoi':
        total += 10
        tokens.pop(0)
    elif len(tokens) >= 2 and tokens[0] in digits and tokens[1] == 'muoi':
        total += digits[tokens.pop(0)] * 10
        tokens.pop(0)
    if len(tokens) == 1 and tokens[0] in digits:
        return total + digits[tokens[0]]
    if not tokens and total:
        return total
    raise ValueError('Không đọc được số lượng; hãy nói rõ số và đơn vị.')


def validate_motion(data):
    if not isinstance(data, dict) or set(data) != {'intent', 'actions'} or data['intent'] != 'motion':
        raise ValueError('Định dạng lệnh chuyển động không hợp lệ.')
    actions = data['actions']
    if not isinstance(actions, list) or not 1 <= len(actions) <= MAX_ACTIONS:
        raise ValueError('Chuỗi phải có từ 1 đến 12 bước.')
    clean = []
    for index, action in enumerate(actions):
        if not isinstance(action, dict):
            raise ValueError('Action phải là object.')
        kind = action.get('type')
        if kind == 'stop' and set(action) == {'type'} and len(actions) == 1:
            clean.append(dict(action))
            continue
        allowed = ('left', 'right') if kind in ('rotate', 'rotate_continuous') else ('forward', 'backward')
        if kind not in ('rotate', 'rotate_continuous', 'move', 'navigate_move') or action.get('direction') not in allowed:
            raise ValueError('Loại action hoặc hướng không hợp lệ.')
        if kind == 'navigate_move' and len(actions) != 1:
            raise ValueError('Điều hướng né vật cản phải là một lệnh riêng.')
        params = set(action) - {'type', 'direction'}
        if kind == 'rotate_continuous':
            if params or index != len(actions) - 1:
                raise ValueError('Xoay liên tục phải là bước cuối cùng.')
        else:
            units = (('revolutions', 'degrees') if kind == 'rotate' else
                     ('distance_meters',) if kind == 'navigate_move' else ('duration_seconds', 'distance_meters'))
            if len(params) != 1 or not params.issubset(units):
                raise ValueError('Mỗi bước cần đúng một số lượng và đơn vị phù hợp.')
            key = next(iter(params))
            value = action[key]
            if type(value) not in (int, float) or not 0 < value <= LIMITS[key] or not math.isfinite(value):
                raise ValueError(f'{key} phải dương và không vượt quá {LIMITS[key]}.')
        clean.append(dict(action))
    return {'intent': 'motion', 'actions': clean}


def parse_motion(text):
    """Return None for conversation/waypoints; reject malformed motion imperatives."""
    text = normalize(text)
    text = re.sub(r'^(?:be son|bson|haha|ha ha|khang)\b[ ,:]*', '', text)
    text = re.sub(r'^(?:hay |vui long )', '', text)
    text = re.sub(r' (?:nhe|nha)$', '', text)
    if text in ('dung', 'dung lai', 'dung robot', 'huy lenh', 'huy hanh trinh',
                'stop', 'stop robot', 'stop moving', 'cancel', 'cancel navigation'):
        return {'intent': 'motion', 'actions': [{'type': 'stop'}]}
    if not re.match(r'^(?:xoay|quay(?: sang)? (?:trai|phai)|di thang|di tien|tien|di lui|lui)\b', text):
        return None
    # Questions and quoted/hypothetical requests never authorize actuation.
    if '?' in text or re.search(r'\b(?:la gi|nghia la|tai sao|co duoc khong|duoc khong|the nao)\b', text):
        return None
    actions = []
    for clause in re.split(r'\s+(?:roi|sau do|tiep theo)\s+', text):
        rotation = re.fullmatch(r'(?:xoay|quay)(?: sang)? (trai|phai)(?: (.*))?', clause)
        move = re.fullmatch(r'(di thang|di tien|tien|di lui|lui)(?: (.*))?', clause)
        if rotation:
            direction, amount = rotation.groups()
            action = {'type': 'rotate', 'direction': 'left' if direction == 'trai' else 'right'}
            if amount in ('den khi toi bao dung', 'cho den khi toi bao dung', 'lien tuc'):
                action['type'] = 'rotate_continuous'
                actions.append(action)
                continue
            units = {'vong': 'revolutions', 'do': 'degrees'}
        elif move:
            direction, amount = move.groups()
            action = {'type': 'move', 'direction': 'backward' if 'lui' in direction else 'forward'}
            units = {'giay': 'duration_seconds', 'met': 'distance_meters', 'm': 'distance_meters'}
            if amount and amount.endswith(' ne vat can'):
                action['type'] = 'navigate_move'
                amount = amount[:-len(' ne vat can')]
                units = {'met': 'distance_meters', 'm': 'distance_meters'}
        else:
            raise ValueError('Lệnh chuyển động chưa rõ. Hãy nói hướng, số lượng và đơn vị.')
        match = re.fullmatch(r'(.+) (' + '|'.join(units) + r')', amount or '')
        if not match:
            raise ValueError('Hãy nói rõ số vòng/độ khi xoay, hoặc số giây/mét khi đi.')
        if move and units[match[2]] == 'distance_meters':
            action['type'] = 'navigate_move'
        action[units[match[2]]] = number(match[1])
        actions.append(action)
    return validate_motion({'intent': 'motion', 'actions': actions})
