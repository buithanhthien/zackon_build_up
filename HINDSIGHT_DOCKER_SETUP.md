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

## Routine operation

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
