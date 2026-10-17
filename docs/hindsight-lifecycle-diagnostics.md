# Hindsight lifecycle diagnostics

The UI owns Hindsight through `robot_ui/hindsight_lifecycle.py`. Startup uses
Compose `up -d --wait --wait-timeout 120 hindsight`. A separate 300-second
command deadline also bounds image downloads and daemon stalls. Closing the UI
cancels startup and runs `stop --timeout 15 hindsight` with a 25-second deadline.
A failed startup also triggers cleanup immediately.

Docker output is captured in a private temporary file and mapped to fixed error
categories. Neither output, exception text, environment, nor interpolated Compose
configuration is printed. Messages distinguish startup failure from cleanup
failure and include command exit codes or explicit timeout categories. Unknown
errors remain generic; inspect service logs privately on the target machine.
An unsupported `--wait` is reported explicitly: readiness must not be inferred
from a successful detached start without a health check.

## Host findings, 2026-10-12

- Docker CLI and daemon: 29.1.3. Compose: 2.40.3; both wait flags are supported.
- `docker compose -f docker-compose.hindsight.yml config --quiet` passed.
- Project `.env` contains a nonempty `OPENAI_API_KEY` that does not match common
  placeholder patterns. This does not verify that the provider accepts the key.
- The current `khoaiuh` process lacks supplementary group 125 (`docker`), although
  `/etc/group` already lists this user in that group. The Docker socket is
  `root:docker`, mode 660. Docker access fails with permission denied in this
  process, including outside the sandbox.
- Running read-only diagnostics through `sg docker` succeeds against the daemon.
  Initially there was no `hindsight` container and neither 8888 nor 9999 was
  listening, so there were no service logs or health results to inspect.

Log out and log in again, then reopen the IDE and launch the UI so it inherits
the updated group membership. Check `id` in that launch environment. A new shell
using `newgrp docker` can also receive the group; launch the UI from that shell.
There is no need to change socket permissions or add the user to the group again.

## Verification

Run `python3 -m unittest discover -s tests -p test_hindsight_lifecycle.py`.
The mocks cover readiness, cancellation, missing executable, Compose variables,
daemon access, unsupported wait flags, unhealthy containers, health/start timeout,
command timeout, stop errors, and keeping startup and cleanup messages separate.
They also check that secret-like strings in Docker errors are never reported.

For desktop acceptance, launch the UI in a session with Docker access, wait for
`Bộ nhớ lâu dài đã sẵn sàng`, close it normally, then confirm the cleanup message
and container state. Preserve the named volume throughout; do not use `down -v`
or any volume removal/pruning command.

The real lifecycle was also run using the user's existing `docker` group via
`sg docker`. Startup hit the 300-second command deadline without reporting ready;
cleanup `stop` returned exit 0. Interim inspection still found no container.
This verifies the deadline and cleanup path, not service readiness. Image
availability/download and startup need further checking in the refreshed login
session. Desktop open/close acceptance has not been completed.

Final checks still found no container and no listeners on 8888/9999. Inspecting
the configured image locally failed; a read-only registry manifest inspection
also timed out after 20 seconds. These observations point to image/registry
access as the next investigation branch, without proving a particular network,
authentication, or registry fault. No provider request was verified.

## UI report and memory routing fix, 2026-10-14

The operator's UI logs now show readiness and successful cleanup. However, those
messages alone do not confirm retention of a chat fact. The first reply paraphrased
the supplied identity without an explicit persistence result; the next session
selected `web`, which previously bypassed memory entirely.

Chat memory now waits for the UI-owned lifecycle startup result before recall.
The web route checks user-taught memory first. The planner distinguishes required
external verification/current evidence from ordinary identity questions using
`external_required`. Ordinary lookups can return recalled facts; explicit external
verification still searches, with recalled identity context marked unverified.
Web probes cannot write corrections. If recall is unavailable, an ordinary web
probe reports the outage instead of silently substituting a namesake result.
Web instructions also require identity disambiguation before discussing adverse
claims about a person with the same name.

Memory operation results are preserved in the final reply, including mixed tasks.
Safe diagnostics report recall availability/count, decision category, and
`retain=confirmed` or `retain=session_only`, without printing stored facts.
A direct factual introduction is included in the memory classifier's teaching
instructions; questions and quoted instructions must not be retained.

Regression tests simulate teaching in one session and recall through a new memory
object with empty session history, plus startup waiting/failure, read-only web
probes, explicit verification, and outage behavior. They use mocked storage and
model decisions, not a live provider. Desktop acceptance still needs a fresh
teaching turn with `retain=confirmed`, normal close, then the same identity question
in a new UI session. Do not infer that the earlier ambiguous acknowledgement saved
the original fact; re-teach it if recall has no matching record.

### Follow-up: teaching was silently skipped

The next operator log conclusively shows `recall=available records=0`, then
`action=fallback`, with no retain operation. This is a skipped write, not proof
of data loss on restart. The extraction prompt still contained a contradictory
empty-memory fallback rule. That rule now applies only to non-teaching requests.
The planner's `memory_task` is forwarded as `teaching_task`; teaching extraction
uses an action schema restricted to `correct` or `clarify`. Unexpected decisions
and missing results explicitly say that nothing was saved rather than silently
acknowledging. Questions and read-only probes must still not write.

Recall failures now log fixed categories (connection, timeout, HTTP authentication,
HTTP client/server, invalid response, startup not ready), never raw responses.
Memory requests and ordinary local misses preserve an outage warning instead of
silently replacing unavailable memory with a web answer. Current direct API checks,
including outside the sandbox, failed with `connection`; that does not establish
the cause of the earlier logged outage.

Compose child commands use separate sessions so terminal Ctrl+C does not directly
interrupt cleanup. Exit 130 in the supplied log is consistent with SIGINT and is
separate from the missing teaching write.

Validation: 87 unit tests passed. A live model check with synthetic fictional data
selected memory_task=true, action=correct and invoked a mocked retain against an
empty mocked bank. This validates model routing/extraction, not real persistence.
The proposed live check containing the supplied personal identity was rejected by
automatic approval review before execution; synthetic data was used instead.
No memory volume or stored records were deleted or modified by these checks.

### Retain timeout: preserve pending teaching across UI restarts

The subsequent log reaches `action=correct` but the synchronous retain request
exceeds its 120-second deadline. The old fallback stored the fact only in RAM,
so restarting lost that fallback. A timeout does not prove whether the server
committed; it must remain an unconfirmed result.

Before sending retain, enabled memory now commits a private SQLite outbox record
in `.local/hindsight-pending.sqlite3` (excluded from Git). The directory and file
are created with modes 700 and 600. `HINDSIGHT_PENDING_PATH` can override the path.
Records are partitioned by the configured Hindsight URL/bank and preserve the
original document ID, scope, provenance and revisions. A restarted UI loads pending
facts and can answer from them even if recall fails. On a successful recall during
a non-teaching turn, it retries at most one pending record, once per document per
process; this retry may still take up to 120 seconds. Confirmed records are removed
from the outbox, with a value comparison preventing an older acknowledgement from
deleting a newer local edit. Failed retries remain on disk. No timed-out request
is reported as a confirmed Hindsight save.

User messages distinguish local pending storage from Hindsight confirmation. A
local disk failure falls back honestly to session-only storage. A pure memory
turn returns the memory result directly, removing the second model-generated
paraphrase that duplicated acknowledgements. Mixed tasks still use composition.
The existing Hindsight volume is untouched. Previously lost RAM-only facts are
not reconstructed from logs; the operator must teach them again once.

Read-only host diagnosis found that PostgreSQL permission errors were from
2026-10-12. The 2026-10-14 runs show both local embedding/reranker initialization
and application startup completing. Current volume directories belong to the
container user (1000:1000), so no ownership changes were made. An unauthenticated
HTTPS probe from `hindsight_default` to OpenAI returned 401, proving basic network
reachability but not model/key compatibility or successful retain processing.
The server-side timeout cause remains unconfirmed; increasing the timeout alone
was not used as a fix. No live personal-data write was performed during diagnosis.

Regression coverage uses actual temporary SQLite files for timeout/restart,
offline recall, stable-ID replay, failed replay, disk errors, bank isolation,
concurrent edits, and ensuring read-only questions create no outbox.
