"""Private durable outbox for user-taught facts awaiting Hindsight confirmation."""

from contextlib import closing
import json
import os
from pathlib import Path
import sqlite3


class PendingMemory:
    def __init__(self, path, namespace):
        self.path = Path(path)
        self.namespace = namespace

    def _connect(self):
        self.path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        # Create with private permissions before sqlite opens it.
        fd = os.open(self.path, os.O_CREAT | os.O_RDWR, 0o600)
        os.close(fd)
        connection = sqlite3.connect(self.path, timeout=5)
        connection.execute('CREATE TABLE IF NOT EXISTS pending ('
                           'namespace TEXT, document_id TEXT, record TEXT NOT NULL, '
                           'PRIMARY KEY(namespace, document_id))')
        return connection

    def load(self):
        if not self.path.exists():
            return {}
        with closing(self._connect()) as connection:
            rows = connection.execute('SELECT document_id, record FROM pending WHERE namespace=?',
                                      (self.namespace,)).fetchall()
        records = {}
        for document_id, encoded in rows:
            record = json.loads(encoded)
            if (not isinstance(record, dict) or record.get('document_id') != document_id
                    or not isinstance(record.get('text'), str)
                    or not isinstance(record.get('metadata'), dict)):
                raise ValueError('Invalid pending memory record')
            records[document_id] = record
        return records

    def save(self, record):
        with closing(self._connect()) as connection, connection:
            connection.execute('INSERT OR REPLACE INTO pending VALUES (?, ?, ?)',
                               (self.namespace, record['document_id'],
                                json.dumps(record, ensure_ascii=False)))

    def acknowledge(self, record):
        # A concurrent newer edit must not be deleted by an older acknowledgement.
        with closing(self._connect()) as connection, connection:
            connection.execute('DELETE FROM pending WHERE namespace=? AND document_id=? AND record=?',
                               (self.namespace, record['document_id'],
                                json.dumps(record, ensure_ascii=False)))
