"""Build and run the fft_axi xsim testbench (Vivado 2026.1).

    python xsim.py build          generate the IP model, compile, elaborate
    python xsim.py run <dir> [1]  run one prepared case (1: DOUBLE_BUF build)

test_fft.py uses build() and run(); the xfft IP model is generated once
into build/ (delete build/ip to regenerate it)."""
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
FFT = os.path.join(ROOT, "fft")
BUILD = os.path.join(HERE, "build")
VIVADO = os.environ.get("XILINX_VIVADO", "/opt/Xilinx/2026.1/Vivado")
RTL = [os.path.join(FFT, "rtl", f) for f in ("fft_win.sv", "fft_acc.sv", "fft_axi.sv")] + \
      [os.path.join(ROOT, "dvbs2", "rtl", "dvbs2_async_fifo.sv")]
WIN_MEM = os.path.join(FFT, "rtl", "gen", "fft_win.mem")
SNAP = {0: "tb_fft_sb", 1: "tb_fft_db"}


def _env():
    env = dict(os.environ)
    b = os.path.join(VIVADO, "bin")
    if b not in env.get("PATH", "").split(":"):
        env["PATH"] = b + ":" + env.get("PATH", "")
    env.setdefault("XILINX_VIVADO", VIVADO)
    return env


def _run(cmd, cwd, log):
    with open(os.path.join(cwd, log), "w") as f:
        r = subprocess.run(cmd, cwd=cwd, env=_env(), stdout=f, stderr=subprocess.STDOUT)
    if r.returncode:
        raise RuntimeError(f"{cmd[0]} failed, see {os.path.join(cwd, log)}")


def build():
    os.makedirs(BUILD, exist_ok=True)
    sh = os.path.join(BUILD, "export", "fft_xfft", "xsim", "fft_xfft.sh")
    if not os.path.exists(sh):
        _run([os.path.join(VIVADO, "bin", "vivado"), "-mode", "batch", "-nojournal", "-nolog",
              "-source", os.path.join(HERE, "build_ip.tcl"), "-tclargs", BUILD], BUILD, "build_ip.log")
    libs = re.search(r"xelab .*?(-L xil_defaultlib.*?)--snapshot", open(sh).read()).group(1).split()
    _run(["xvhdl", "--relax", "-work", "xil_defaultlib", os.path.join(BUILD, "ip", "fft_xfft", "sim", "fft_xfft.vhd")], BUILD, "xvhdl.log")
    _run(["xvlog", "-sv", "--relax", "-work", "xil_defaultlib", "-L", "xpm"] + RTL + [os.path.join(HERE, "tb_fft.sv"),
          os.path.join(VIVADO, "data", "verilog", "src", "glbl.v")], BUILD, "xvlog.log")
    for db, snap in SNAP.items():
        _run(["xelab", "--relax", "--mt", "8", "--debug", "off"] + libs +
             ["-L", "unisims_ver", "-L", "unimacro_ver", "-generic_top", f"DOUBLE_BUF={db}",
              "--snapshot", snap, "xil_defaultlib.tb_fft", "xil_defaultlib.glbl"], BUILD, f"xelab_{snap}.log")


def run(case_dir, double_buf=0, adc_n=0, tap_n=1, tap_div=1):
    shutil.copy(WIN_MEM, os.path.join(case_dir, "fft_win.mem"))
    # a private copy of the snapshot: xsim writes the run's plusargs into it
    snap = os.path.join(case_dir, "xsim.dir", SNAP[double_buf])
    shutil.rmtree(snap, ignore_errors=True)
    shutil.copytree(os.path.join(BUILD, "xsim.dir", SNAP[double_buf]), snap)
    _run(["xsim", SNAP[double_buf], "-R",
          "--testplusarg", f"DIR={case_dir}", "--testplusarg", f"ADC_N={adc_n}",
          "--testplusarg", f"TAP_N={tap_n}", "--testplusarg", f"TAP_DIV={tap_div}"], case_dir, "xsim.log")
    log = open(os.path.join(case_dir, "xsim.log")).read()
    if "FATAL" in log or "DONE at" not in log:
        raise RuntimeError(f"simulation failed, see {case_dir}/xsim.log")
    return log


if __name__ == "__main__":
    if sys.argv[1] == "build":
        build()
    else:
        d = os.path.abspath(sys.argv[2])
        n = [int(x) for x in open(os.path.join(d, "sizes.txt")).read().split()]
        print(run(d, int(sys.argv[3]) if len(sys.argv) > 3 else 0, *n))
