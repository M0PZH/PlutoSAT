#!/usr/bin/python3
"""plutosat-boot: put the PlutoSAT boot files on a fishball board's SD card
boot partition (FAT, /dev/mmcblk0p1), defensively.

  plutosat-boot check      [--fat-dir DIR]   preflight only, change nothing
  plutosat-boot install    [--fat-dir DIR] [--dry-run] [--force]
  plutosat-boot rollback   [--fat-dir DIR] [--force]   (= plutosat-boot-rollback)
  plutosat-boot status     [--fat-dir DIR]

install writes, for the files whose content changes:
  BOOT.bin        the packaged one (FSBL + the PlutoSAT bitstream + u-boot):
                  the PL is configured before the kernel starts
  devicetree.dtb  the board's CURRENT one through dtb-add-plutosat.py
                  --pl-ranges (reserved memory, UIO nodes; no IIO DMAs and no
                  PL node the bitstream does not have, e.g. mwipcore@43c00000)
  uEnv.txt        + the Debian "uenvcmd=" line when it has none (nothing else
                  changes)
Before that it keeps the current three files as
NAME.pre-plutosat-VERSION-DATE and as NAME.plutosat-prev (the set
plutosat-boot-rollback restores). Each file is written to a temporary name,
fsynced, renamed, and read back from the card (page cache dropped) and
compared by SHA-256. Nothing changes when every file is already as it would
be. It never reboots and never writes the QSPI flash.

Preflight (install refuses on any failure unless --force or
PLUTOSAT_BOOT_FORCE=1):
  - the board is a fishball Zynq-7020 (device tree model / compatible);
  - the boot partition is a FAT file system with BOOT.bin, devicetree.dtb
    and uEnv.txt, and has room for the new files and the backups;
  - every PL node (0x40000000..0xBFFFFFFF) of the resulting device tree lies
    in the bitstream's AXI address map (pl-axi.txt), the PlutoSAT nodes are
    there, the IIO DMAs are not;
  - the u-boot environment in QSPI (mtd "qspi-uboot-env"), which the
    fishball u-boot uses instead of its own whenever its CRC is valid,
    boots through uEnv.txt: bootcmd "run $modeboot", modeboot=sdboot,
    sdboot runs uenvboot, uenvboot runs uenvcmd;
  - an existing uenvcmd= line in uEnv.txt boots Debian with this
    devicetree.dtb and the generic-uio bootargs.

--fat-dir DIR works on a directory (a copy of the partition) instead of the
partition, for tests; the board checks still run unless --force.
"""
import argparse
import datetime
import hashlib
import os
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import zlib

SHARE = os.environ.get("PLUTOSAT_BOOT_SHARE", "/usr/share/plutosat-boot")
DEV = "/dev/mmcblk0p1"
FILES = ["BOOT.bin", "devicetree.dtb", "uEnv.txt"]
PREV = ".plutosat-prev"
PL_WINDOW = (0x40000000, 0xC0000000)
# what the PlutoSAT software needs in the device tree
NEED_NODES = [
    "reserved-memory/dvbs2-buffer@3f000000", "reserved-memory/dvbs2-rx-buffer@3ef00000",
    "reserved-memory/nbiq-buffer@3eb00000", "reserved-memory/fft-buffer@3ea00000",
    "reserved-memory/dvbs2-llr-buffer@3e800000", "reserved-memory/iqtx-buffer@3e700000",
    "fpga-axi@0/dvbs2-tx@7c500000", "fpga-axi@0/dvbs2-rx@7c510000", "fpga-axi@0/nbiq@7c520000",
    "fpga-axi@0/rx-awgn@7c530000", "fpga-axi@0/fft@7c540000", "fpga-axi@0/iqtx@7c550000",
]
NO_NODES = ["fpga-axi@0/dma@7c400000", "fpga-axi@0/dma@7c420000", "fpga-axi@0/mwipcore@43c00000"]
UNDO_BY_HAND = """\
By hand (another computer, the SD card's first partition, FAT):
  copy BOOT.bin.plutosat-prev       over BOOT.bin
  copy devicetree.dtb.plutosat-prev over devicetree.dtb
  copy uEnv.txt.plutosat-prev       over uEnv.txt
(or any older set: NAME.pre-plutosat-VERSION-DATE, NAME.pre-rollback-DATE)."""


class Refuse(Exception):
    pass


def sha(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def sha_bytes(b):
    return hashlib.sha256(b).hexdigest()


def read_uncached(path):
    """The file as the card has it: drop its pages from the cache first."""
    fd = os.open(path, os.O_RDONLY)
    try:
        os.posix_fadvise(fd, 0, 0, os.POSIX_FADV_DONTNEED)
        chunks = []
        while True:
            b = os.read(fd, 1 << 20)
            if not b:
                break
            chunks.append(b)
        return b"".join(chunks)
    finally:
        os.close(fd)


def fsync_dir(d):
    try:
        fd = os.open(d, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    except OSError:
        pass   # vfat: directory fsync is not always supported; sync() follows


def write_verified(d, name, data):
    """Write d/name through a temporary name, fsync, rename, read back."""
    tmp = os.path.join(d, "." + name + ".plutosat-tmp")
    dst = os.path.join(d, name)
    with open(tmp, "wb") as f:
        f.write(data)
        f.flush()
        os.fsync(f.fileno())
    os.rename(tmp, dst)
    fsync_dir(d)
    os.sync()
    got = read_uncached(dst)
    if sha_bytes(got) != sha_bytes(data):
        raise RuntimeError(f"{dst}: read back differs from what was written (SHA-256)")


# ---------------------------------------------------------------- board checks

def check_board(fails):
    try:
        model = open("/proc/device-tree/model", "rb").read().rstrip(b"\0").decode(errors="replace")
        compat = open("/proc/device-tree/compatible", "rb").read().decode(errors="replace").split("\0")
    except OSError as e:
        fails.append(f"no device tree model ({e}): not a fishball board?")
        return
    if "FISH Ball" not in model or "Z7020" not in model or "xlnx,zynq-7000" not in compat:
        fails.append(f"this is not a fishball Zynq-7020 board (model \"{model}\", compatible {compat})")


def mtd_by_name(name):
    try:
        for line in open("/proc/mtd"):
            m = re.match(r'(mtd\d+):\s+([0-9a-f]+)\s+[0-9a-f]+\s+"([^"]*)"', line)
            if m and m.group(3) == name:
                return "/dev/" + m.group(1) + "ro", int(m.group(2), 16)
    except OSError:
        pass
    return None, 0


def check_uboot_env(fails, notes, path=None):
    """The QSPI environment the fishball u-boot prefers to its own."""
    if path is None:
        path, size = mtd_by_name("qspi-uboot-env")
        if path is None:
            fails.append("no mtd \"qspi-uboot-env\" in /proc/mtd: the u-boot environment in QSPI cannot be checked")
            return
    try:
        with open(path, "rb") as f:
            data = f.read()
    except OSError as e:
        fails.append(f"cannot read the u-boot environment ({path}): {e}")
        return
    crc, = struct.unpack_from("<I", data, 0)
    if zlib.crc32(data[4:]) & 0xFFFFFFFF != crc:
        notes.append(f"u-boot environment in QSPI ({path}): no valid CRC, u-boot uses its built-in one (fine)")
        return
    env = {}
    for e in data[4:].split(b"\0\0")[0].split(b"\0"):
        k, _, v = e.decode(errors="replace").partition("=")
        if k:
            env[k] = v
    bad = []
    if not re.search(r"run \$\{?modeboot\}?", env.get("bootcmd", "")):
        bad.append(f"bootcmd is \"{env.get('bootcmd', '(unset)')}\", not \"run $modeboot\"")
    if env.get("modeboot") != "sdboot":
        bad.append(f"modeboot is \"{env.get('modeboot', '(unset)')}\", not \"sdboot\"")
    if "run uenvboot" not in env.get("sdboot", ""):
        bad.append("sdboot does not run uenvboot")
    if "uenvcmd" not in env.get("uenvboot", "") or "loadbootenv" not in env.get("uenvboot", ""):
        bad.append("uenvboot does not load uEnv.txt and run uenvcmd")
    if bad:
        fails.append(
            f"the u-boot environment in QSPI ({path}, valid CRC) would not boot through uEnv.txt: "
            + "; ".join(bad) + ". It is probably another firmware's. Reset it at the u-boot prompt "
            "(serial console, interrupt the autoboot): env default -a; saveenv  -- then boot once and "
            "run this again. This tool does not write the flash.")
    else:
        notes.append(f"u-boot environment in QSPI ({path}): valid, boots through uEnv.txt")


# ---------------------------------------------------------------- the partition

class Fat:
    """The boot partition: where it is mounted, or mounted by us read-only
    (read-write only once something is written: writable())."""

    def __init__(self, fat_dir=None):
        self.ours = None
        self.ro = False
        if fat_dir:
            self.dir = fat_dir
            return
        if not os.path.exists(DEV):
            raise Refuse(f"no {DEV} (SD card boot partition)")
        t = subprocess.run(["blkid", "-o", "value", "-s", "TYPE", DEV], capture_output=True, text=True).stdout.strip()
        if t not in ("vfat", "msdos"):
            raise Refuse(f"{DEV} is not a FAT file system (blkid: \"{t}\")")
        real = os.path.realpath(DEV)
        for line in open("/proc/mounts"):
            f = line.split()
            if os.path.realpath(f[0]) == real:
                self.ro = "rw" not in f[3].split(",")
                self.dir = f[1].replace("\\040", " ")
                return
        self.ours = tempfile.mkdtemp(prefix="plutosat-boot.", dir="/run")
        self.ro = True
        r = subprocess.run(["mount", "-t", "vfat", "-o", "ro", DEV, self.ours], capture_output=True, text=True)
        if r.returncode:
            os.rmdir(self.ours)
            self.ours = None
            raise Refuse(f"cannot mount {DEV}: {r.stderr.strip()}")
        self.dir = self.ours

    def writable(self):
        if not self.ro:
            return
        if not self.ours:
            raise Refuse(f"{DEV} is mounted read-only at {self.dir} (not by plutosat-boot): remount it read-write")
        r = subprocess.run(["mount", "-o", "remount,rw", self.ours], capture_output=True, text=True)
        if r.returncode:
            raise Refuse(f"cannot remount {DEV} read-write: {r.stderr.strip()}")
        self.ro = False

    def path(self, name):
        return os.path.join(self.dir, name)

    def free(self):
        s = os.statvfs(self.dir)
        return s.f_bavail * s.f_frsize

    def close(self):
        if self.ours:
            os.sync()
            subprocess.run(["umount", self.ours])
            try:
                os.rmdir(self.ours)
            except OSError:
                pass
            self.ours = None


# ---------------------------------------------------------------- device tree

def dts_of(dtb_path):
    return subprocess.run(["dtc", "-q", "-I", "dtb", "-O", "dts", dtb_path], check=True,
                          capture_output=True, text=True).stdout


def read_ranges(path):
    out = []
    for line in open(path):
        f = line.split("#")[0].split()
        if f:
            out.append((int(f[0], 16), int(f[1], 16), f[2] if len(f) > 2 else "?"))
    return out


def node_paths(dts):
    """Every node's path, e.g. fpga-axi@0/iqtx@7c550000 (top level: no /)."""
    stack, out = [], []
    for line in dts.splitlines():
        m = re.match(r"^(\t*)([^\s{}]+) \{$", line)
        if m:
            depth = len(m.group(1))
            del stack[depth:]
            stack.append(m.group(2))
            out.append("/".join(stack[1:]))
    return out


def check_dt(dts, ranges, fails):
    paths = node_paths(dts)
    for p in paths:
        leaf = p.rsplit("/", 1)[-1]
        if "@" not in leaf:
            continue
        try:
            a = int(leaf.split("@")[1].split(",")[0], 16)
        except ValueError:
            continue
        if PL_WINDOW[0] <= a < PL_WINDOW[1] and not any(lo <= a <= hi for lo, hi, _ in ranges):
            fails.append(f"device tree node {p} is not in the bitstream (a PL node at 0x{a:08x}); booting with the PL loaded would hang")
    for n in NEED_NODES:
        if n not in paths:
            fails.append(f"device tree lacks {n}")
    for n in NO_NODES:
        if n in paths:
            fails.append(f"device tree still has {n}")


def new_dtb(cur_path, work):
    """(bytes or None if unchanged, message)."""
    out = os.path.join(work, "devicetree.dtb")
    r = subprocess.run([sys.executable, os.path.join(SHARE, "dtb-add-plutosat.py"), "--pl-ranges",
                        os.path.join(SHARE, "pl-axi.txt"), cur_path, out], capture_output=True, text=True)
    if r.returncode == 3:
        return None, "unchanged"
    if r.returncode:
        raise Refuse(f"dtb-add-plutosat.py: {(r.stderr or r.stdout).strip()}")
    return open(out, "rb").read(), r.stdout.strip()


# ---------------------------------------------------------------- uEnv.txt

def new_uenv(cur, fails):
    """uEnv.txt with the Debian uenvcmd= line (None: unchanged)."""
    line = open(os.path.join(SHARE, "uenvcmd.txt"), "rb").read().strip()
    text = cur.decode("latin-1")
    have = [ln for ln in text.splitlines() if ln.startswith("uenvcmd=")]
    if have:
        u = have[-1]
        if "${devicetree_image}" not in u or "uio_pdrv_genirq.of_id=generic-uio" not in u or "bootm" not in u:
            fails.append("uEnv.txt has a uenvcmd= line of its own that does not boot Debian with "
                         "${devicetree_image} and uio_pdrv_genirq.of_id=generic-uio; merge by hand:\n  "
                         + line.decode())
        return None
    eol = b"\r\n" if b"\r\n" in cur else b"\n"
    sep = b"" if cur.endswith(b"\n") or not cur else eol
    return cur + sep + line + eol


# ---------------------------------------------------------------- commands

def version():
    try:
        return open(os.path.join(SHARE, "version")).read().strip()
    except OSError:
        return "unknown"


def plan(args, fat, fails, notes, work):
    """{name: new bytes} for the files that change."""
    for n in FILES:
        if not os.path.isfile(fat.path(n)):
            fails.append(f"no {n} on the boot partition ({fat.dir})")
    if fails:
        return {}
    changes = {}
    boot = open(os.path.join(SHARE, "BOOT.bin"), "rb").read()
    if sha(fat.path("BOOT.bin")) != sha_bytes(boot):
        changes["BOOT.bin"] = boot
    dtb, msg = new_dtb(fat.path("devicetree.dtb"), work)
    if dtb is not None:
        changes["devicetree.dtb"] = dtb
        notes.append("devicetree.dtb: " + msg)
        open(os.path.join(work, "check.dtb"), "wb").write(dtb)
        check_dt(dts_of(os.path.join(work, "check.dtb")), read_ranges(os.path.join(SHARE, "pl-axi.txt")), fails)
    else:
        check_dt(dts_of(fat.path("devicetree.dtb")), read_ranges(os.path.join(SHARE, "pl-axi.txt")), fails)
    u = new_uenv(open(fat.path("uEnv.txt"), "rb").read(), fails)
    if u is not None:
        changes["uEnv.txt"] = u
    # room: the new files plus two backups of each current file
    need = sum(len(b) for b in changes.values()) + 2 * sum(os.path.getsize(fat.path(n)) for n in FILES) + (1 << 20)
    if changes and fat.free() < need:
        fails.append(f"the boot partition has {fat.free() >> 10} KiB free, {need >> 10} KiB needed")
    return changes


def preflight(args, fails, notes):
    if args.board_checks:
        check_board(fails)
        check_uboot_env(fails, notes, args.uboot_env)


def report(fails, notes, force):
    for n in notes:
        print("plutosat-boot: " + n)
    for f in fails:
        print(("plutosat-boot: WARNING (forced): " if force else "plutosat-boot: REFUSED: ") + f, file=sys.stderr)


def cmd_check(args):
    fails, notes = [], []
    preflight(args, fails, notes)
    fat = Fat(args.fat_dir)
    try:
        with tempfile.TemporaryDirectory() as work:
            changes = plan(args, fat, fails, notes, work)
    finally:
        fat.close()
    report(fails, notes, False)
    if not fails:
        print("plutosat-boot: preflight passed; "
              + ("would change: " + ", ".join(changes) if changes else "the boot files are up to date"))
    return 1 if fails else 0


def cmd_install(args):
    fails, notes = [], []
    preflight(args, fails, notes)
    fat = Fat(args.fat_dir)
    try:
        with tempfile.TemporaryDirectory() as work:
            changes = plan(args, fat, fails, notes, work)
            report(fails, notes, args.force)
            if fails and not args.force:
                print("plutosat-boot: nothing changed. Fix the above, or PLUTOSAT_BOOT_FORCE=1 (at your own risk: "
                      "a board that does not boot needs someone at the SD card).", file=sys.stderr)
                return 1
            if not changes:
                print(f"plutosat-boot: the boot files on {fat.dir} are already those of plutosat-boot {version()}: nothing to do")
                return 0
            if args.dry_run:
                print("plutosat-boot: dry run, would change: " + ", ".join(changes))
                return 0
            fat.writable()
            stamp = datetime.datetime.now().strftime("%Y%m%dT%H%M%S")
            tag = f".pre-plutosat-{version()}-{stamp}"
            old = {n: open(fat.path(n), "rb").read() for n in FILES}
            for n in FILES:
                write_verified(fat.dir, n + tag, old[n])
                write_verified(fat.dir, n + PREV, old[n])
            print(f"plutosat-boot: kept the current files as *{tag} and *{PREV} on {fat.dir}")
            for n in ["devicetree.dtb", "uEnv.txt", "BOOT.bin"]:
                if n in changes:
                    write_verified(fat.dir, n, changes[n])
                    print(f"plutosat-boot: wrote {n} (sha256 {sha_bytes(old[n])[:12]} -> {sha_bytes(changes[n])[:12]}, verified)")
            print(f"""plutosat-boot: done; it takes effect at the next boot (this tool never reboots).
Changed: {", ".join(changes)}.
To undo, before rebooting or after a boot that works:  plutosat-boot-rollback
{UNDO_BY_HAND}""")
            return 0
    finally:
        fat.close()


def cmd_rollback(args):
    fails, notes = [], []
    if args.board_checks:
        check_board(fails)
    report(fails, notes, args.force)
    if fails and not args.force:
        return 1
    fat = Fat(args.fat_dir)
    try:
        prev = [n for n in FILES if os.path.isfile(fat.path(n + PREV))]
        if not prev:
            print(f"plutosat-boot: no *{PREV} files on {fat.dir}: nothing to roll back to", file=sys.stderr)
            return 1
        data = {n: open(fat.path(n + PREV), "rb").read() for n in prev}
        todo = [n for n in prev if not os.path.isfile(fat.path(n)) or sha(fat.path(n)) != sha_bytes(data[n])]
        if not todo:
            print(f"plutosat-boot: the boot files already are the *{PREV} set: nothing to do")
            return 0
        fat.writable()
        stamp = datetime.datetime.now().strftime("%Y%m%dT%H%M%S")
        for n in todo:
            if os.path.isfile(fat.path(n)):
                write_verified(fat.dir, n + ".pre-rollback-" + stamp, open(fat.path(n), "rb").read())
        for n in ["devicetree.dtb", "uEnv.txt", "BOOT.bin"]:
            if n in todo:
                write_verified(fat.dir, n, data[n])
                print(f"plutosat-boot: restored {n} from {n}{PREV} (verified)")
        print(f"plutosat-boot: rolled back; the files replaced are kept as *.pre-rollback-{stamp}. "
              "It takes effect at the next boot.")
        return 0
    finally:
        fat.close()


def cmd_status(args):
    fat = Fat(args.fat_dir)
    try:
        boot = sha(os.path.join(SHARE, "BOOT.bin"))
        for n in FILES:
            p = fat.path(n)
            s = sha(p) if os.path.isfile(p) else None
            extra = ""
            if n == "BOOT.bin" and s:
                extra = " (plutosat-boot's)" if s == boot else " (not plutosat-boot's)"
            print(f"{n:16} {s[:16] if s else 'missing'}{extra}")
            pp = p + PREV
            if os.path.isfile(pp):
                print(f"{n + PREV:30} {sha(pp)[:16]}")
        return 0
    finally:
        fat.close()


def main():
    ap = argparse.ArgumentParser(prog="plutosat-boot", description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("command", choices=["check", "install", "rollback", "status"])
    ap.add_argument("--fat-dir", help="work on this directory instead of the boot partition (tests)")
    ap.add_argument("--dry-run", action="store_true", help="install: show what would change")
    ap.add_argument("--force", action="store_true", default=os.environ.get("PLUTOSAT_BOOT_FORCE") == "1",
                    help="go on despite failed checks (also PLUTOSAT_BOOT_FORCE=1)")
    ap.add_argument("--uboot-env", help=argparse.SUPPRESS)          # a file instead of the mtd (tests)
    ap.add_argument("--no-board-checks", dest="board_checks", action="store_false", help=argparse.SUPPRESS)
    args = ap.parse_args()
    if os.geteuid() != 0 and not args.fat_dir:
        sys.exit("plutosat-boot: run as root")
    try:
        return {"check": cmd_check, "install": cmd_install, "rollback": cmd_rollback, "status": cmd_status}[args.command](args)
    except Refuse as e:
        print(f"plutosat-boot: REFUSED: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
