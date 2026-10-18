# Hindsight on the robot PC

Hindsight runs as a local Docker service. Its database is stored in the Docker
named volume `hindsight-data`, outside the Git checkout, so rebuilding or
updating the repository does not remove saved memories.

## First-time setup

1. Install Docker Engine and the Docker Compose plugin on the robot PC.
2. Copy the settings from `hindsight.env.example` into the project-root `.env`.
   If `.env` already contains `OPENAI_API_KEY`, keep it and add only the
   `HINDSIGHT_API_LLM_MODEL` setting. Never commit `.env`.
3. From the project root, start the service:

   ```bash
   docker compose -f docker-compose.hindsight.yml up -d
   ```

4. Check the service and open its local control panel:

   ```bash
   docker compose -f docker-compose.hindsight.yml ps
   docker compose -f docker-compose.hindsight.yml logs -f hindsight
   ```

   API: `http://127.0.0.1:8888`  
   Control panel: `http://127.0.0.1:9999`

The ports are bound to loopback so other devices on the network cannot connect
to the Hindsight API or control panel by default. The Hindsight API uses TCP
8888; the robot's micro-ROS agent uses UDP 8888, so these are separate
transports.

## Upgrading Hindsight

The Compose file pins Hindsight `1.0.0`. Before its first start with an existing
`hindsight-data` volume, make a backup: the service applies database migrations
at startup. Keep the existing volume; do not use `down -v` when upgrading.

From the project root, pull and start the pinned release:

```bash
docker compose -f docker-compose.hindsight.yml pull hindsight
docker compose -f docker-compose.hindsight.yml up -d --wait --wait-timeout 120 hindsight
```

The application uses the same bank and API endpoints after the upgrade.

## Robot correction memory

`ChatPanel` now checks the dedicated `beson-corrections` bank before its
existing IUH database/web route. Explicit corrections and requests to remember
facts are saved synchronously. Ordinary web answers are not saved. A subsequent
correction can replace the recalled document for the same subject/attribute.
Ambiguous corrections trigger a follow-up question rather than a write.

Defaults work with the existing `.env`: `HINDSIGHT_ENABLED=true`,
`HINDSIGHT_BANK_ID=beson-corrections`. The UI derives the local API URL from
`HINDSIGHT_API_PORT` (8888 by default); `HINDSIGHT_URL` overrides it.
Set `HINDSIGHT_ENABLED=false` and restart the UI to disable persistent storage.
Correction recognition remains active: each ChatPanel keeps corrections in
memory across its chat turns, even if Hindsight is disabled or unavailable.
Closing that panel or restarting the application loses these session-only facts.
The robot distinguishes session acknowledgement from a confirmed long-term save.
Unsaved session facts are not automatically synchronized when Hindsight returns;
explicitly repeat the correction to save it after reconnecting.
An unavailable bank does not block ordinary questions: the local/web route
can proceed. Memory operations report unconfirmed saves; ordinary answers are
not automatically prefixed with a memory warning.
A save timeout means the outcome is unknown, not necessarily that nothing was
written; check the control panel before retrying.

This setup trusts the current speaker to teach facts. It does not authenticate
an administrator. Use separate bank IDs for separate deployments/users when
needed; do not use this as a public, access-controlled knowledge editor.
Fact extraction, matching an existing document, and retrieval use LLM/semantic
inference, so they require live evaluation with your actual Vietnamese speech.
Memory and local lookup turns make an additional correction-routing model call,
including when persistent storage is disabled. General tasks and explicit web
lookups skip it unless the planner also detects a memory update.

Manual acceptance check (requires running Docker and valid provider access):

1. Ask where a fictional teacher's room is.
2. Say explicitly: "Please remember: teacher Test A's room is X5.7."
3. Wait for the saved confirmation, then ask the room again.
4. Correct it to X5.8 and verify the next answer is X5.8.
5. Restart the UI and container without removing the volume, then ask again.
6. Stop Hindsight; correct a fact and ask again within the same panel. Verify it
   acknowledges only session storage and uses the corrected fact.
7. Ask an unrelated question; verify the usual answer route continues.
8. Repeat steps 6-7 with `HINDSIGHT_ENABLED=false` after restarting the UI.
   Session corrections should still work, without connection attempts.

Offline regression tests (mocked LLM/HTTP, no API usage):

```bash
python3 -m unittest discover -s tests -v
```

API contract checked against the pinned version:
https://github.com/vectorize-io/hindsight/blob/v1.0.0/hindsight-api-slim/hindsight_api/api/http.py

### Troubleshooting a forgotten fact

Record the exact acknowledgement, time, API address and bank for each attempt.
“Mình chưa xác nhận lưu được vào bộ nhớ lâu dài” means the save is unconfirmed;
an answer in the same panel only proves session memory works.

The UI reads the project-root `.env`, but existing process environment variables
take precedence. Check `HINDSIGHT_ENABLED`, `HINDSIGHT_URL`,
`HINDSIGHT_API_PORT` and `HINDSIGHT_BANK_ID` in the launcher environment too.
With `HINDSIGHT_ENABLED=false`, there are no Hindsight requests at all.
After starting a healthy service, set it to `true`, restart the UI and teach the
fact again. Never print the full environment or expanded Compose configuration:
they can contain API keys. Review container logs locally before sharing excerpts.

In the control panel, select the same bank and inspect the source document ID,
original content and creation/update timestamps. Test recall with both the
original question and a paraphrase. For fictional facts, explicitly ask about
the fictional scenario; test facts cannot override real IUH JSON evidence.
Separately test an operator-supplied correction of one known IUH JSON field.

The client sends the planner's resolved retrieval question, without the chat
history, to recall. Hindsight 1.0.0 returns recall metadata directly. For older
responses that omit it, the client recovers metadata from the matching document
listing's `retain_params.metadata` before applying scope/provenance filters.
That compatibility lookup adds HTTP requests per recalled document only when
metadata is missing. If retrieval or metadata lookup fails, the correction
remains session-only; repeat the explicit correction after recovery. Local
lookup misses now check memory before web.

For persistence acceptance, use `docker compose -f docker-compose.hindsight.yml
restart hindsight`, wait for health, reopen the UI, then repeat both questions.
Inspect the container's mounts and Compose project if a document disappears:
the project is `hindsight`, the volume key is `hindsight-data`, and the mount is
`/home/hindsight/.pg0`. Do not remove volumes during diagnosis.

Use this record for each original/paraphrased query before and after restart:

| Time / phase | Exact acknowledgement or answer | Bank | Document ID / created / updated | Recall returned it? | UI used it? |
| --- | --- | --- | --- | --- | --- |
| Teach / ask / correct / restart / ask | | | | | |

If there is no document, inspect recognition and synchronous retain. If the
document exists but recall misses it, inspect query/bank/retrieval. If recall
finds it but UI ignores it, inspect metadata, scope and answer routing. Mock
tests do not establish Docker persistence or Vietnamese model quality.

## Local IUH lookup and correction overlay

`robot_ui/iuh_database.json` remains the repository-managed source of IUH data.
The chat route now searches that JSON deterministically through
`robot_ui/iuh_local_search.py` and keeps the selected JSON path with each local
evidence item. The LLM may phrase the final answer, but it no longer decides by
itself whether the whole JSON database is sufficient.

The local search object checks the JSON file modification time before every
search. If an operator edits `iuh_database.json` while the UI is running, the
next chat lookup reloads the file automatically after the edit has been saved.

A correction that targets one local JSON fact is stored as an overlay, not
written back into `iuh_database.json`. Its Hindsight metadata includes the exact
`json_path` and the JSON value that was present when the correction was taught
(`base_json_value`). Only a correction with the same path and the same base value
may override local evidence. Corrections for other paths are filtered out before
they are shown to the correction-routing LLM.

Each scoped JSON path uses one stable correction document ID. Re-teaching the
same path replaces that correction instead of creating parallel versions. If the
repository value later changes, the old correction becomes stale and no longer
overrides the repository value.

Questions that clearly request fresh information (for example latest news,
weather, prices, or current information) bypass the local answer and use web
search. Ambiguous local questions ask for clarification rather than guessing. If
Hindsight is unavailable, local JSON lookup and web fallback continue normally;
only persisted corrections from earlier sessions may be unavailable.

Regression tests for this routing can be run with:

```bash
python3 -m unittest -v \
  tests/test_iuh_local_search.py \
  tests/test_correction_scope.py \
  tests/test_correction_memory.py
```

## Start and stop

Docker owns the service lifecycle independently of Startup Layout. Enable boot
startup once on a systemd host:

```bash
sudo systemctl enable --now docker.service containerd.service
docker compose -f docker-compose.hindsight.yml up -d --wait --wait-timeout 120
```

The Compose policy is `always`: **OFF lasts until the next Docker Engine restart
or machine reboot**, when Hindsight starts again. Merely reopening the UI does
not turn it on. No separate boot job should run Compose stop/down. A removed
container cannot restart; run the setup command again after removing it.

Startup Layout shows “Chế độ học” / “Learning mode” beside the hardware cards.
It inspects Docker every five seconds, shows ON only for a healthy running
container, and provides Enable/Disable buttons. Commands run outside Qt's main
thread and repeated clicks are blocked while busy. Errors use fixed diagnostic
messages; raw Docker output and container environment are never logged. Closing
the UI detaches the monitor and leaves the container and any accepted operation
alone. The named volume is preserved by Disable (`compose stop`).

The embedded PostgreSQL 18.1 installation in `hindsight-data` includes its ICU
70 runtime libraries. The Hindsight image may not include that ICU version, so
Compose sets `LD_LIBRARY_PATH` to the installation's `lib` directory. Keep this
setting aligned with the PostgreSQL version stored in the volume; do not delete
the installation or volume as a workaround.

At boot, Hindsight waits for a TCP connection to `api.openai.com:443` before
starting its API. This avoids repeated failed starts when Docker comes up before
the host has working external network access. If the connection never becomes
available, check the host network and firewall; an unhealthy service can also
indicate a separate provider credential or configuration problem.

`HINDSIGHT_ENABLED=true` remains required for chat to use storage. This card
controls the local container; a custom remote `HINDSIGHT_URL` is not controlled.
The existing readiness callback follows the current observed health, including
recovery after enabling. Session and pending-memory fallback remain available.

Acceptance on the robot PC (do not reboot during robot operation):

1. Reboot without opening Startup Layout; verify the container becomes healthy.
2. Open the UI: it should show Checking then ON, without sending Compose up.
3. Disable: wait for OFF; close/reopen the UI and verify it stays OFF.
4. Enable: wait through Starting to ON. Repeated clicks must not enqueue commands.
5. Stop the container externally; the card must update within a polling cycle.
6. With Docker unavailable or an unhealthy container, verify an error/unknown
   state and no false successful transition; restore Docker and verify recovery.
7. Disable, reboot, and verify Hindsight returns to ON without opening the UI.
8. Confirm existing saved memories remain after all these steps.

Policy reference: https://docs.docker.com/engine/containers/start-containers-automatically/
Boot setup: https://docs.docker.com/engine/install/linux-postinstall/

The API key is passed to the Hindsight container for its memory-processing LLM
calls. This configuration keeps the memory database on the robot PC, while
those LLM requests still go to OpenAI. Hindsight uses local embedding and
reranking models in the full image. The full image is large; check available
disk space and RAM on the robot PC before pulling it.
