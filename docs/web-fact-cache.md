# Web fact cache

Web evidence lives in `.local/web-facts.sqlite3`, independently of IUH JSON and
Hindsight's pending-memory database. Each operation opens/closes its own SQLite
connection; files are created with mode 0600. `WEB_FACT_CACHE_PATH` overrides the
path. Delete this file to clear the cache.

`WEB_FACT_CACHE_TTLS` is a JSON object of TTLs in seconds. Defaults: `room` and
`personnel` 86400; `curriculum` 2592000; `history` 31536000; `other` 604800.
For example: `{"room":3600,"personnel":21600}`. TTL changes apply to new writes.
Expiry is measured from collection, not publication. Publication dates remain
separate metadata; collecting a 2022 page today does not verify its currentness.

After live search, up to three actual source URLs are fetched (public HTTPS,
no redirects, HTML/plain text, bounded size and timeout). The extractor receives
only fetched page text and the retrieval question, never the generated answer.
The cache rejects facts without a literal supporting passage containing subject,
value and identifying scope, and rejects dates absent from the page. It stores
subject, attribute, value, scope, source URL/title/date, quote, provenance=web,
collection time and expiry. It never writes web facts into user correction memory.
Extraction and relevance still depend on model judgment: literal containment is
an evidence guard, not a proof of semantic entailment.

Each fact is validated independently, including when extraction reports an incomplete
answer. Invalid facts are skipped with reason counts; valid facts are merged without
silently overwriting conflicting source values. Each fact expires independently.
Lookup groups facts by literal subject and scope, independently of the stored
question. Full names support broad introductions and role questions. A short given
name requires a literal personnel role (for example “Tâm giáo vụ”); multiple matching
identities produce a miss. Explicit scope and unknown qualifiers must match stored
scope/attribute/value tokens. This conservative Vietnamese vocabulary is not semantic
search; unfamiliar phrasing may miss. Translation/story requests are not name lookups.
All supported facts for a matched identity reach composition, which selects relevant
attributes and reports gaps. General-route cache misses retain normal dialogue.
A cache hit can contain only part of an answer: composition must state remaining gaps.
Explicit online verification or a current-state request bypasses cache; common
Vietnamese/English phrases have a deterministic guard in addition to routing.

Diagnostics: `python3 -m robot_ui.web_fact_cache` prints the absolute database path,
total stored facts and unexpired facts without creating an absent database.
`python3 -m robot_ui.web_fact_cache --query "Đặng Thanh Tâm"` also returns
matching records. Enable INFO logging for `robot_ui.web_fact_cache` to see extracted/saved counts, rejection
reasons and cache hits. Fetch failures and write errors include exception details;
failed optional cache writes do not discard successful web answers.

Composition receives web metadata, scoped recalled user records and available
local evidence. Latest explicit user corrections take priority for the same
attribute; local JSON precedes web evidence. Conflicts must retain attribution and
source dates. Old page values must not be described as verified current values.
Existing correction-memory scope checks remain in place for repository overrides.

Offline regression checks:

```
python3 -m unittest discover -s tests -p 'test_web*.py'
python3 -m unittest discover -s tests -p 'test_conversation*.py'
python3 -m unittest discover -s tests -p 'test_correction_memory.py'
```

These use mocked model responses and source pages. They verify storage, expiry,
routing, metadata transport and rejection of unsupported evidence; they do not
replace an end-to-end evaluation of live model wording or university pages.

Live-source smoke check (2026-10-15): fetched the public IUH electrical faculty
staff page `https://feet.iuh.edu.vn/category.html%40220%40Nhan-vien-co-huu`
using `fetch_source`, selected its literal passage for Đặng Thanh Tâm and stored
one personnel fact (Giáo vụ) in the default database. No publication date was
inferred. A repeat/paraphrase hit the cache; an explicit current-room request
invoked the web callback. Answer callbacks were mocked: this checks real source
fetch/storage and routing, not an end-to-end model/UI conversation. The page lists
X5.1-4; X5.2.5 was not stored as source-confirmed information.

Follow-up retrieval check (2026-10-15): the existing default database returned its
single unexpired personnel record for “Cho mình biết thêm thông tin về cô Đặng
Thanh Tâm”. The stored value was “Giáo vụ”. The regression suites passed 104 tests
(web, conversation and correction patterns). Composition tests inspect evidence
and policy sent to a mocked model; they do not certify generated wording. A fresh
live web/model/UI conversation remains to be checked.
