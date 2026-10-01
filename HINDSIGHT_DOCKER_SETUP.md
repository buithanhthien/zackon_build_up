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
An unavailable bank no longer blocks ordinary questions: the local/web route
can proceed, with a warning that corrections from earlier sessions are unavailable.
A save timeout means the outcome is unknown, not necessarily that nothing was
written; check the control panel before retrying.

This setup trusts the current speaker to teach facts. It does not authenticate
an administrator. Use separate bank IDs for separate deployments/users when
needed; do not use this as a public, access-controlled knowledge editor.
Fact extraction, matching an existing document, and retrieval use LLM/semantic
inference, so they require live evaluation with your actual Vietnamese speech.
There is one additional model call per chat turn for correction recognition,
including when persistent storage is disabled.

Manual acceptance check (requires running Docker and valid provider access):

1. Ask where a fictional teacher's room is.
2. Say explicitly: "Please remember: teacher Test A's room is X5.7."
3. Wait for the saved confirmation, then ask the room again.
4. Correct it to X5.8 and verify the next answer is X5.8.
5. Restart the UI and container without removing the volume, then ask again.
6. Stop Hindsight; correct a fact and ask again within the same panel. Verify it
   acknowledges only session storage and uses the corrected fact.
7. Ask an unrelated question; verify local/web answering continues with a warning.
8. Repeat steps 6-7 with `HINDSIGHT_ENABLED=false` after restarting the UI.
   Session corrections should still work, without connection attempts.

Offline regression tests (mocked LLM/HTTP, no API usage):

```bash
python3 -m unittest discover -s tests -v
```

API contract checked against the pinned version:
https://github.com/vectorize-io/hindsight/blob/v0.4.9/hindsight-api/hindsight_api/api/http.py

## Start and stop

The container restarts automatically after a Docker/PC restart once it has been
started the first time. To stop it manually:

```bash
docker compose -f docker-compose.hindsight.yml down
```

This leaves the named volume and its memories intact. To remove the service and
all saved Hindsight memories, the volume must be removed separately:

```bash
docker compose -f docker-compose.hindsight.yml down --volumes
```

The API key is passed to the Hindsight container for its memory-processing LLM
calls. This configuration keeps the memory database on the robot PC, while
those LLM requests still go to OpenAI. Hindsight uses local embedding and
reranking models in the full image. The full image is large; check available
disk space and RAM on the robot PC before pulling it.
