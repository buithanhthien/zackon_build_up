#!/usr/bin/env python3
"""Process manager for tracking and cleaning up subprocesses"""
import subprocess
import signal
from typing import Optional, List


class ProcessManager:
    """Manages subprocess lifecycle with proper cleanup"""
    
    def __init__(self):
        self._processes: List[subprocess.Popen] = []
    
    def launch_terminal(self, command: str, description: str = "") -> Optional[subprocess.Popen]:
        """Launch a command in gnome-terminal and track it
        
        Args:
            command: Bash command to execute
            description: Optional description for logging
            
        Returns:
            Popen object or None if failed
        """
        try:
            proc = subprocess.Popen([
                'gnome-terminal', '--', 'bash', '-c', command
            ])
            self._processes.append(proc)
            return proc
        except Exception as e:
            print(f"[ProcessManager] Failed to launch {description}: {e}")
            return None
    
    def kill_by_pattern(self, pattern: str, signal_type=signal.SIGTERM):
        """Kill processes matching a pattern using pkill
        
        Args:
            pattern: Pattern to match (passed to pkill -f)
            signal_type: Signal to send (default SIGTERM)
        """
        try:
            subprocess.run(['pkill', f'-{signal_type}', '-f', pattern], check=False)
        except Exception as e:
            print(f"[ProcessManager] Failed to kill pattern '{pattern}': {e}")
    
    def cleanup_all(self):
        """Terminate all tracked processes"""
        for proc in self._processes:
            if proc.poll() is None:  # Still running
                try:
                    proc.terminate()
                    proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    proc.kill()
                except Exception:
                    pass
        self._processes.clear()
    
    def remove_finished(self):
        """Remove finished processes from tracking list"""
        self._processes = [p for p in self._processes if p.poll() is None]
    
    def __del__(self):
        """Cleanup on deletion"""
        self.cleanup_all()
