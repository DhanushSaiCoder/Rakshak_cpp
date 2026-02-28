# tegrastats_reader.py
import json
import re
import subprocess
from typing import Dict, Optional

class TegrastatsReader:
    CPU_RE = re.compile(r"CPU\s*\[(.*?)\]")
    GR3D_RE = re.compile(r"GR3D_FREQ\s+(\d+)%")
    EMC_RE = re.compile(r"EMC_FREQ\s+(\d+)%")
    RAM_RE = re.compile(r"RAM\s+(\d+)\/(\d+)MB")
    SWAP_RE = re.compile(r"SWAP\s+(\d+)\/(\d+)MB")
    TEMP_TOKEN_RE = re.compile(r"([A-Za-z0-9_]+)@(\d+(?:\.\d+)?)C")

    def __init__(self, interval_ms: int = 1000):
        self.interval_ms = interval_ms
        self.proc: Optional[subprocess.Popen] = None

    def start(self):
        if self.proc:
            return
        self.proc = subprocess.Popen(
            ["tegrastats", "--interval", str(self.interval_ms)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

    def stop(self):
        if not self.proc:
            return
        try:
            self.proc.terminate()
            self.proc.wait(timeout=2)
        except Exception:
            try:
                self.proc.kill()
            except Exception:
                pass
        self.proc = None

    def read_one(self) -> Dict[str, Optional[str]]:
        if not self.proc or not self.proc.stdout:
            raise RuntimeError("TegrastatsReader not started")

        line = self.proc.stdout.readline()
        if not line:
            raise RuntimeError("tegrastats produced no output")
        line = line.strip()

        data: Dict[str, Optional[str]] = {
            "cpu_usage_overall_pct": None,
            "gpu_usage_overall_pct": None,
            "cpu_temp_c": None,
            "gpu_temp_c": None,
            "emc_pct": None,
            "ram_used_mb": None,
            "ram_total_mb": None,
            "swap_used_mb": None,
            "swap_total_mb": None,
        }

        # CPU overall = average of per-core %
        m = self.CPU_RE.search(line)
        if m:
            cores = [c.strip() for c in m.group(1).split(",")]
            vals = []
            for c in cores:
                pct = c.split("%")[0]
                try:
                    vals.append(float(pct))
                except Exception:
                    pass
            if vals:
                data["cpu_usage_overall_pct"] = f"{sum(vals)/len(vals):.1f}"

        # GPU %
        m = self.GR3D_RE.search(line)
        if m:
            data["gpu_usage_overall_pct"] = m.group(1)

        # EMC %
        m = self.EMC_RE.search(line)
        if m:
            data["emc_pct"] = m.group(1)

        # RAM / SWAP
        m = self.RAM_RE.search(line)
        if m:
            data["ram_used_mb"], data["ram_total_mb"] = m.group(1), m.group(2)

        m = self.SWAP_RE.search(line)
        if m:
            data["swap_used_mb"], data["swap_total_mb"] = m.group(1), m.group(2)

        # Temps (normalize to lowercase keys: cpu, gpu, tj, soc0, ...)
        temps = {name.lower(): temp for (name, temp) in self.TEMP_TOKEN_RE.findall(line)}

        # These will now work with your lines like cpu@51.187C gpu@46.656C
        data["cpu_temp_c"] = temps.get("cpu")
        data["gpu_temp_c"] = temps.get("gpu")

        return data
