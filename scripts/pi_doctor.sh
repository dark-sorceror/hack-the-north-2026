#!/usr/bin/env bash
# pi_doctor.sh -- "why can't I reach the Pi?", in plain language, in 20 s at most.
#
#   scripts/pi_doctor.sh                                   # hao@hao.local, bridge :7777
#   scripts/pi_doctor.sh --host fe80::8aa2:9eff:fea9:d084%en9
#   scripts/pi_doctor.sh --host pi@retriever-pi.local --port 7777 --iface en9
#
# Walks the path from the laptop to the bridge one hop at a time -- the USB LAN
# adapter and its cable, Internet Sharing, mDNS, ping, port 22, key auth, the
# bridge port -- and prints one line per check plus a one-line fix for each
# failure. The last line is a single VERDICT.
#
# It never hangs: macOS has no `timeout`, and nc/ssh/dscacheutil can all sit
# forever on a dead link-local address, so every probe runs under a hard SIGALRM
# (see bounded) and the whole run is capped by --budget (default 20 s).
#
# Exit status: 0  Pi reachable over SSH and the bridge answers
#              3  Pi reachable over SSH, bridge not answering (scripts/pi_deploy.sh --run)
#              1  Pi not usable -- the VERDICT line says why
#              2  usage error
#
# Env defaults: PI_HOST PI_USER PI_PORT PI_IFACE PI_DOCTOR_BUDGET.
# scripts/pi_deploy.sh sources this file for bounded, tcp_probe and PI_SSH_OPTS;
# sourcing only defines functions and variables.

# ---------------------------------------------------------------- helpers ---

# One set of ssh options for every call: never prompt, give up fast on a dead
# link, notice a link that dies mid-session, trust a Pi seen for the first time
# (but still refuse one whose key CHANGED), and keep the chatter out.
PI_SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=5 -o ServerAliveInterval=3
             -o ServerAliveCountMax=2 -o StrictHostKeyChecking=accept-new
             -o LogLevel=ERROR)

# bounded SECS CMD [ARGS...] -- run CMD; after SECS seconds, SIGTERM it (SIGKILL
# a second later) and return 142 (128 + SIGALRM, what `timeout` would say).
# perl forks and waits, so a hung child cannot hold us, the child can clean up
# (rsync takes its ssh down with it), and bash never sees a job die of a signal
# (it would print "Alarm clock: 14"). stdin/stdout/stderr pass straight through.
bounded() {
  local secs=$1
  shift
  case $secs in '' | *[!0-9]* | 0) secs=1 ;; esac
  perl -e '
    use POSIX ":sys_wait_h";
    my $t = shift @ARGV;
    my $pid = fork;
    exit 127 unless defined $pid;
    if ($pid == 0) { exec { $ARGV[0] } @ARGV; exit 127 }
    $SIG{ALRM} = sub {
      kill "TERM", $pid;
      for (1 .. 10) { exit 142 if waitpid($pid, WNOHANG) == $pid; select(undef, undef, undef, 0.1) }
      kill "KILL", $pid; waitpid($pid, 0); exit 142;
    };
    alarm $t;
    while (waitpid($pid, 0) != $pid) {}
    exit(($? & 127) ? 128 + ($? & 127) : $? >> 8);
  ' "$secs" "$@"
}

# tcp_probe HOST PORT SECS -- connect, read the first line the server sends.
# Prints exactly one of:
#   OPEN <first line, or nothing if the server sent nothing within 2 s>
#   REFUSED | TIMEOUT | UNREACH <why> | ERROR <why>
# Returns 0 OPEN, 1 REFUSED, 2 TIMEOUT, 3 UNREACH, 4 ERROR. Uses perl's
# IO::Socket::IP (core perl), which takes scoped addresses like fe80::1%en9 --
# and unlike nc it can tell "refused" (nothing listening) from "timeout" (no
# route, host gone), which is most of the diagnosis.
tcp_probe() {
  local out rc
  out=$(bounded $(($3 + 1)) perl -e '
    use strict; use Socket qw(SOCK_STREAM); use IO::Socket::IP;
    use Errno qw(ECONNREFUSED ETIMEDOUT EHOSTUNREACH ENETUNREACH EHOSTDOWN);
    my ($h, $p, $t) = @ARGV;
    my $s = IO::Socket::IP->new(PeerHost => $h, PeerPort => $p,
                                Type => SOCK_STREAM, Timeout => $t);
    unless ($s) {
      my $n = $! + 0; my $e = "$!"; my $at = $@; $at =~ s/\s+\z//;
      if ($n == ECONNREFUSED) { print "REFUSED\n"; exit 1 }
      if ($n == ETIMEDOUT)    { print "TIMEOUT\n"; exit 2 }
      if ($n == EHOSTUNREACH || $n == ENETUNREACH || $n == EHOSTDOWN) {
        print "UNREACH $e\n"; exit 3 }
      print "ERROR ", ($at ne "" ? $at : $e), "\n"; exit 4;
    }
    my $line = "";
    eval { local $SIG{ALRM} = sub { die "t\n" }; alarm($t < 2 ? $t : 2);
           $line = <$s>; alarm 0; 1 };
    $line = "" unless defined $line; $line =~ s/[\r\n]+\z//;
    print "OPEN ", substr($line, 0, 200), "\n"; exit 0;
  ' "$1" "$2" "$3" 2>/dev/null)
  rc=$?
  if [ "$rc" -eq 142 ]; then
    echo TIMEOUT
    return 2
  fi
  [ -n "$out" ] || out="ERROR probe failed (exit $rc)"
  echo "$out"
  return "$rc"
}

# is_ip ADDR -- true for a literal IPv4 or IPv6 address (not a host name).
is_ip() {
  case $1 in
    *:*) return 0 ;;
    *[!0-9.]*) return 1 ;;
    *.*.*.*) return 0 ;;
  esac
  return 1
}

# scoped_v6 ADDR -- dscacheutil prints link-local addresses in the kernel's KAME
# form, the interface index embedded in the second group (fe80:19::1 is
# fe80::1 on the interface with scopeid 0x19). Give back fe80::1%en9.
scoped_v6() {
  local a=$1 sc idx ifn
  if [[ $a == *%* ]]; then
    echo "$a"
    return
  fi
  if [[ $a =~ ^[fF][eE]80:([0-9a-fA-F]+)::(.*)$ ]]; then
    sc=${BASH_REMATCH[1]}
    idx=$(printf '%x' "$((16#$sc))")
    ifn=$(ifconfig -a 2>/dev/null | awk -v want="scopeid 0x$idx" '
      /^[a-z]/ { i = $1; sub(":$", "", i) }
      index($0, want) { print i; exit }')
    a="fe80::${BASH_REMATCH[2]}"
    [ -n "$ifn" ] && a="$a%$ifn"
  fi
  echo "$a"
}

# resolve_host NAME SECS -- print the addresses NAME resolves to, one per line,
# link-local ones scoped (fe80::...%en9). Nothing printed = did not resolve.
resolve_host() {
  local name=$1 secs=$2
  if [ "$(uname -s)" = Darwin ]; then
    # Goes through mDNSResponder, like ssh does. Blocks ~forever on a name
    # nobody answers for, hence bounded.
    bounded "$secs" dscacheutil -q host -a name "$name" 2>/dev/null |
      awk '/^ip_address:|^ipv6_address:/ { print $2 }' |
      while read -r a; do
        case $a in *:*) scoped_v6 "$a" ;; *) echo "$a" ;; esac
      done | awk '!seen[$0]++'
  else
    bounded "$secs" getent ahosts "$name" 2>/dev/null | awk '!seen[$1]++ { print $1 }'
  fi
}

# link_neighbors IFACE SECS "MAC ADDRS" -- who else answers a multicast ping
# on the cable (space-separated), whatever their names or addresses. The Mac
# answers its own ping, so -c 3 ends as soon as its own three replies are in;
# a live Pi answers inside the same window.
link_neighbors() {
  local a others
  others=$(bounded "$2" ping6 -c 3 -i 0.3 "ff02::1%$1" 2>/dev/null |
    awk '/bytes from/ { a = $4; sub(",$", "", a); print a }' | awk '!seen[$0]++' |
    while read -r a; do
      [[ " $3 " == *" $a "* ]] || echo "$a"
    done | tr '\n' ' ')
  echo "${others% }"
}

# ------------------------------------------------------------------ doctor ---

_d_color() {
  if [ -t 1 ] && [ -z "${NO_COLOR:-}" ]; then
    D_OK=$'\033[32m[ ok ]\033[0m' D_WARN=$'\033[33m[warn]\033[0m'
    D_BAD=$'\033[31m[FAIL]\033[0m' D_INFO=$'\033[2m[info]\033[0m' D_SKIP=$'\033[2m[skip]\033[0m'
  else
    D_OK='[ ok ]' D_WARN='[warn]' D_BAD='[FAIL]' D_INFO='[info]' D_SKIP='[skip]'
  fi
}

# _say ok|warn|fail|info|skip MESSAGE [FIX] -- one check line (+ its fix).
# The first failure along the path becomes the verdict's reason.
_say() {
  local tag
  case $1 in
    ok) tag=$D_OK ;; warn) tag=$D_WARN ;; fail) tag=$D_BAD ;; info) tag=$D_INFO ;; *) tag=$D_SKIP ;;
  esac
  printf '  %s %s\n' "$tag" "$2"
  if [ -n "${3:-}" ]; then
    printf '         fix: %s\n' "$3"
  fi
  if [ "$1" = fail ] && [ -z "$D_FAIL" ]; then
    D_FAIL=$2
    D_FIX=${3:-}
  fi
  return 0
}

# _cap N -- N, or what is left of the budget if less (0 = out of time).
_cap() {
  local left=$((D_BUDGET - (SECONDS - D_T0)))
  [ "$left" -lt 0 ] && left=0
  if [ "$1" -lt "$left" ]; then echo "$1"; else echo "$left"; fi
}

# _nm_check INFO -- the Pi's eth0 is managed by NetworkManager with DHCP
# ("auto") and no DHCP server answers on the direct cable: NM gives up after
# 45 s, tears eth0 down -- its IPv6 link-local address with it, a ~2 s
# blackout that trips the bridge watchdog -- and starts over; after 4 failures
# it waits 5 minutes with eth0 down. Seen live on the Pi. Sets D_NM.
_nm_check() {
  local state conn ip
  state=$(printf '%s\n' "$1" | sed -n 's/^pi_nm_state=//p')
  conn=$(printf '%s\n' "$1" | sed -n 's/^pi_nm_conn=//p')
  ip=$(printf '%s\n' "$1" | sed -n 's/^pi_nm_ip=//p')
  ip=${ip% }
  D_NM=''
  case $state in
    *connecting* | *"IP configuration"*)
      if [[ $ip == auto* || $ip == *" auto"* ]]; then
        D_NM="the Pi's NetworkManager is still waiting for DHCP on eth0 ('$conn': $ip): it drops eth0 every 45 s (a ~2 s blackout that trips the watchdog) and for 5 min after 4 tries"
        _say warn "$D_NM" "$(nm_fix "$conn")"
      fi
      ;;
  esac
  return 0
}

# nm_fix CONN -- the one-line cure for the DHCP cycle: link-local IPv6 always,
# and DHCP that keeps trying without ever failing the connection. Or give the
# cable a DHCP server: Internet Sharing on the Mac, to the USB LAN.
nm_fix() {
  echo "on the Pi: sudo nmcli con mod '${1:-netplan-eth0}' ipv6.method link-local ipv4.dhcp-timeout 2147483647 && sudo nmcli con up '${1:-netplan-eth0}'   (or share Internet to the USB LAN on the Mac)"
}

doctor_usage() {
  cat <<'EOF'
usage: scripts/pi_doctor.sh [--host HOST] [--user USER] [--port PORT] [--iface IFACE] [--budget SECS]

Why can't I reach the Pi? Checks, in order: the USB LAN adapter and its link,
Internet Sharing, name resolution (mDNS), ping, port 22 + SSH banner, SSH key
auth, and the bridge port -- each with a one-line fix -- then one VERDICT line.

  --host HOST     Pi host name or address; USER@HOST also works   (default hao.local)
  --user USER     SSH user                                        (default hao)
  --port PORT     bridge TCP port                                 (default 7777)
  --iface IFACE   the Mac's interface to the Pi (default: auto-detect the USB LAN adapter)
  --budget SECS   hard cap on the whole run                       (default 20)

exit: 0 all good, 3 SSH ok but the bridge is not answering, 1 Pi not usable, 2 usage
EOF
}

doctor_main() {
  local host=${PI_HOST:-hao.local} user=${PI_USER:-hao} port=${PI_PORT:-7777}
  local iface=${PI_IFACE:-}
  D_BUDGET=${PI_DOCTOR_BUDGET:-20}
  while [ $# -gt 0 ]; do
    case $1 in
      --host | --user | --port | --iface | --budget)
        if [ $# -lt 2 ]; then
          echo "pi_doctor: $1 needs a value" >&2
          return 2
        fi
        case $1 in
          --host) host=$2 ;; --user) user=$2 ;; --port) port=$2 ;;
          --iface) iface=$2 ;; --budget) D_BUDGET=$2 ;;
        esac
        shift 2
        ;;
      -h | --help)
        doctor_usage
        return 0
        ;;
      *)
        echo "pi_doctor: unknown option: $1" >&2
        doctor_usage >&2
        return 2
        ;;
    esac
  done
  case $host in *@*) user=${host%%@*} host=${host#*@} ;; esac
  case $port in '' | *[!0-9]*) echo "pi_doctor: bad --port: $port" >&2; return 2 ;; esac
  case $D_BUDGET in '' | *[!0-9]* | 0) echo "pi_doctor: bad --budget: $D_BUDGET" >&2; return 2 ;; esac

  _d_color
  D_T0=$SECONDS D_FAIL='' D_FIX='' D_NORESOLVE='' D_NM=''
  local darwin=0
  [ "$(uname -s)" = Darwin ] && darwin=1
  echo "pi_doctor: $user@$host, bridge port $port (budget ${D_BUDGET}s)"

  # 1. The adapter and the cable -------------------------------------------
  local carrier=unknown mac_addrs=''
  if [ $darwin = 1 ]; then
    if [ -z "$iface" ]; then
      iface=$(bounded 3 networksetup -listallhardwareports 2>/dev/null | awk '
        /^Hardware Port:/ { port = substr($0, 16) }
        /^Device:/ && port ~ /USB.*(LAN|Ethernet)|(LAN|Ethernet).*USB|Thunderbolt Ethernet|AX88|RTL81/ {
          print $2; exit }')
    fi
    if [ -z "$iface" ]; then
      _say fail "no USB LAN adapter found on this Mac" \
        "plug the USB LAN adapter in (or pass --iface enX; list: networksetup -listallhardwareports)"
      carrier=no
    elif ! ifconfig "$iface" >/dev/null 2>&1; then
      _say fail "interface $iface does not exist (adapter unplugged?)" \
        "plug the USB LAN adapter back in; list adapters: networksetup -listallhardwareports"
      carrier=no
    else
      local ifc media status flags
      ifc=$(ifconfig "$iface" 2>/dev/null)
      status=$(printf '%s\n' "$ifc" | awk '/status:/ { print $2 }')
      media=$(printf '%s\n' "$ifc" | sed -n 's/.*media: [a-z]* *(\(.*\)).*/\1/p' | head -1)
      flags=$(printf '%s\n' "$ifc" | head -1)
      mac_addrs=$(printf '%s\n' "$ifc" | awk '$1 == "inet" || $1 == "inet6" { print $2 }' | tr '\n' ' ')
      mac_addrs=${mac_addrs% }
      if [[ $flags != *"<UP,"* && $flags != *",UP,"* ]]; then
        _say fail "$iface is switched off (interface down)" \
          "System Settings > Network > USB 10/100 LAN > turn it on (or: sudo ifconfig $iface up)"
        carrier=no
      elif [ "$status" != active ]; then
        _say fail "$iface: no link -- cable unplugged, bad cable, or the Pi is off" \
          "reseat the cable at both ends; the Pi's Ethernet LEDs should light; check the Pi has power"
        carrier=no
      else
        _say ok "$iface: link up (${media:-unknown speed}); Mac has ${mac_addrs:-no addresses}"
        carrier=yes
      fi
    fi
  else
    _say skip "adapter and Internet Sharing checks are macOS-only"
  fi

  # 2. Internet Sharing ------------------------------------------------------
  if [ $darwin = 1 ] && [ -n "$iface" ]; then
    local members bridges b
    bridges=$(ifconfig -l 2>/dev/null | tr ' ' '\n' | grep '^bridge1[0-9][0-9]$')
    members=''
    for b in $bridges; do
      members="$members $(ifconfig "$b" 2>/dev/null | awk '/member:/ { print $2 }' | tr '\n' ' ')"
    done
    members=$(echo $members)
    if [ -z "$bridges" ]; then
      _say info "Internet Sharing is off: the Pi gets no IPv4 and no internet (IPv6 link-local still works)" \
        "only needed for apt on the Pi: System Settings > General > Sharing > Internet Sharing, share to USB 10/100 LAN"
    elif [[ " $members " == *" $iface "* ]]; then
      _say ok "Internet Sharing serves $iface: the Pi should get a 192.168.2.x address by DHCP"
    else
      _say warn "Internet Sharing is on but shares to ${members:-nothing}, not $iface: the Pi gets no IPv4 and no internet" \
        "System Settings > General > Sharing > Internet Sharing (i) > 'To devices using': tick only USB 10/100 LAN, then toggle sharing off and on"
    fi
  fi

  # 3. Name -> addresses ------------------------------------------------------
  local addrs='' a v4='' v6='' c
  if is_ip "$host"; then
    addrs=$host
    _say ok "$host is a literal address (no name lookup needed)"
  else
    c=$(_cap 3)
    if [ "$c" -lt 1 ]; then
      _say skip "resolve $host: out of time"
    else
      addrs=$(resolve_host "$host" "$c")
      [ -z "$addrs" ] && D_NORESOLVE=$c
    fi
  fi
  for a in $addrs; do
    case $a in
      *:*) [ -z "$v6" ] && v6=$a ;;
      *) [ -z "$v4" ] && v4=$a ;;
    esac
  done
  if [ -n "$addrs" ] && ! is_ip "$host"; then
    local kind
    if [ -n "$v4" ] && [ -n "$v6" ]; then
      kind="IPv4 + IPv6"
    elif [ -n "$v4" ]; then
      kind="IPv4 only"
    elif [[ $v6 == [fF][eE]80:* ]]; then
      kind="IPv6 link-local only, no IPv4"
    else
      kind="IPv6 only, no IPv4"
    fi
    _say ok "$host -> $(echo $addrs) ($kind)"
    if [ -z "$v4" ]; then
      _say info "no IPv4 on the Pi: ssh and the bridge work over IPv6, but only if the bridge binds all addresses (pi_deploy.sh does)"
    fi
  fi

  # 3b. Nothing resolved: is anything alive on the cable at all? That splits
  # "the Pi is off / booting / eth0 down" from "the Pi is up, mDNS is not".
  if [ -n "$D_NORESOLVE" ]; then
    local why="$host does not resolve (mDNS, ${D_NORESOLVE}s)"
    c=$(_cap 3)
    if [ $darwin = 1 ] && [ "$carrier" = yes ] && [ "$c" -ge 1 ]; then
      local others last
      others=$(link_neighbors "$iface" "$c" "$mac_addrs")
      if [ -n "$others" ]; then
        _say fail "$why, but something IS alive on $iface ($others): if that is the Pi, its mDNS (avahi-daemon) is down or still starting" \
          "retry in 10 s, or bypass mDNS: --host $(echo "$others" | awk '{ print $1 }')"
      else
        last=$(ndp -an 2>/dev/null | awk -v i="$iface" '$3 == i && $4 != "permanent" { print $1 " (" $2 ")"; exit }')
        _say fail "$why and nothing on $iface answers but the Mac itself: the Pi is off, still booting, or its Ethernet is down" \
          "check the Pi's power + activity LEDs and its Ethernet port LEDs; a boot takes ~40 s${last:+; last seen: $last}"
      fi
    else
      _say fail "$why: the Pi is not answering on the network" \
        "check the Pi is powered and cabled; wait ~40 s after power-on; or pass its address: --host fe80::...%${iface:-en9}"
    fi
  fi

  # 4. ping -------------------------------------------------------------------
  local target=${v4:-$v6}
  if [ -n "$target" ]; then
    c=$(_cap 2)
    if [ "$c" -lt 1 ]; then
      _say skip "ping: out of time"
    else
      local -a pc
      if [ -n "$v4" ]; then
        pc=(ping -c 1 "$v4")
        [ $darwin = 1 ] && pc=(ping -c 1 -t "$c" "$v4")
      elif [ $darwin = 1 ]; then
        pc=(ping6 -c 1 "$v6")
      else
        pc=(ping -6 -c 1 -W "$c" "$v6")
      fi
      if bounded "$c" "${pc[@]}" >/dev/null 2>&1; then
        _say ok "the Pi answers ping at $target"
      else
        _say warn "no ping reply from $target within ${c}s (the link may be flapping, or the Pi just went down)" \
          "watch it: ${pc[0]} $target   -- replies should be steady before you deploy"
      fi
    fi
  fi

  # 5. Port 22 and the SSH banner -----------------------------------------------
  local sshd=unknown r
  if [ -n "$target" ]; then
    c=$(_cap 3)
    if [ "$c" -lt 1 ]; then
      _say skip "port 22: out of time"
    else
      r=$(tcp_probe "$target" 22 "$c")
      case $r in
        "OPEN SSH-"*)
          sshd=yes
          _say ok "port 22 open, banner: ${r#OPEN }"
          ;;
        OPEN*)
          sshd=yes
          _say warn "port 22 open but no SSH banner within 2 s (sshd overloaded, or the Pi is still booting)" \
            "retry in 10 s"
          ;;
        REFUSED)
          sshd=no
          _say fail "port 22 refused at $target: nothing accepts SSH there (sshd not running, or that is not the Pi)" \
            "on the Pi (keyboard+screen): sudo systemctl enable --now ssh"
          ;;
        TIMEOUT)
          sshd=no
          local why="port 22 at $target: no answer within ${c}s" others=''
          c=$(_cap 3)
          if [ $darwin = 1 ] && [ "$carrier" = yes ] && [ "$c" -ge 1 ]; then
            others=$(link_neighbors "$iface" "$c" "$mac_addrs")
            if [ -z "$others" ]; then
              _say fail "$why, and nothing on $iface answers but the Mac: the Pi is off or rebooting, or NetworkManager took eth0 down after a failed DHCP attempt (it retries within 45 s, or after 5 min)" \
                "wait up to 5 min, or cure it for good -- $(nm_fix)"
            elif [[ " $others " == *" ${target%%\%*}"* ]]; then
              _say fail "$why, yet it answers ping on $iface: sshd is hung or firewalled" \
                "retry in 10 s; if it persists, reboot the Pi"
            else
              _say fail "$why; something else answers on $iface ($others): the Pi's address changed?" \
                "try: scripts/pi_doctor.sh --host $(echo "$others" | awk '{ print $1 }')"
            fi
          elif is_ip "$host"; then
            _say fail "$why: the Pi went away (off, rebooting, or the link dropped)" \
              "check the cable/LEDs and retry in 10 s"
          else
            _say fail "$why: the Pi went away after $host resolved (stale mDNS entry, or the link dropped)" \
              "check the cable/LEDs and retry in 10 s"
          fi
          ;;
        *)
          sshd=no
          _say fail "port 22: ${r#* } (at $target)" \
            "the route to the Pi is broken: reseat the cable; if you passed --host fe80::..., add %${iface:-en9}"
          ;;
      esac
    fi
  fi

  # 6. SSH key auth, and the Pi's own view of itself --------------------------------
  local ssh_ok=0 ssh_host=$host info='' pi_listen=''
  if [ "$sshd" = yes ]; then
    c=$(_cap 8)
    if [ "$c" -lt 2 ]; then
      _say skip "ssh key auth: out of time"
    else
      local errf
      errf=$(mktemp "${TMPDIR:-/tmp}/pi_doctor.XXXXXX")
      info=$(bounded "$c" ssh "${PI_SSH_OPTS[@]}" -o ConnectTimeout=$((c > 5 ? 5 : c)) \
        "$user@$ssh_host" "echo pi_ok=1; echo pi_name=\$(hostname) \$(uname -m);
           ip -br addr 2>/dev/null | awk '\$1 != \"lo\" { print \"pi_if=\" \$0 }';
           ss -ltnH 'sport = :$port' 2>/dev/null | awk '{ print \"pi_listen=\" \$4 }';
           systemctl is-active retriever-bridge 2>/dev/null | sed 's/^/pi_svc=/';
           c=\$(nmcli -g GENERAL.CONNECTION device show eth0 2>/dev/null);
           echo \"pi_nm_state=\$(nmcli -g GENERAL.STATE device show eth0 2>/dev/null)\";
           [ -n \"\$c\" ] && echo \"pi_nm_conn=\$c\" &&
             echo \"pi_nm_ip=\$(nmcli -g ipv4.method,ipv6.method con show \"\$c\" 2>/dev/null | tr '\\n' ' ')\"" \
        2>"$errf" </dev/null)
      r=$?
      local err
      err=$(grep -v '^\*\*' "$errf" | grep -v '^ *$' | head -3 | tr '\n' ' ')
      rm -f "$errf"
      if [ "$r" -eq 0 ] && [[ $info == *pi_ok=1* ]]; then
        ssh_ok=1
        _say ok "ssh $user@$ssh_host: key auth works ($(printf '%s\n' "$info" | sed -n 's/^pi_name=//p'))"
        local ifs
        ifs=$(printf '%s\n' "$info" | sed -n 's/^pi_if=//p' | awk '{ $1 = $1; print }' | tr '\n' ';' | sed 's/;$//; s/;/; /g')
        [ -n "$ifs" ] && _say info "the Pi's interfaces: $ifs"
        pi_listen=$(printf '%s\n' "$info" | sed -n 's/^pi_listen=//p' | tr '\n' ' ')
        pi_listen=$(echo $pi_listen)
        _nm_check "$info"
      elif [ "$r" -eq 142 ]; then
        _say fail "ssh $user@$ssh_host hung for ${c}s after port 22 answered (the link dropped mid-handshake?)" \
          "retry; if it keeps happening the cable/adapter is flapping: ping6 ${v6:-$host} and watch for gaps"
      else
        case $err in
          *"Permission denied"*)
            _say fail "ssh $user@$ssh_host: the Pi rejected this Mac's key" \
              "ssh-copy-id $user@$host   (asks for the Pi's password once)" ;;
          *"IDENTIFICATION HAS CHANGED"* | *"Host key verification failed"*)
            _say fail "ssh $user@$ssh_host: the Pi's host key changed (re-flashed SD card?) -- ssh refuses it" \
              "ssh-keygen -R $host   then run this again" ;;
          *"Could not resolve"*)
            _say fail "ssh could not resolve $host even though it resolved a moment ago: mDNS is flapping" \
              "use the address directly: --host ${v6:-$v4}" ;;
          *"timed out"*)
            _say fail "ssh $user@$ssh_host: connection timed out (link dropped between checks?)" \
              "retry; watch the link with: ping6 ${v6:-$host}" ;;
          *"refused"*)
            _say fail "ssh $user@$ssh_host: connection refused" \
              "on the Pi: sudo systemctl enable --now ssh" ;;
          *)
            _say fail "ssh $user@$ssh_host failed (exit $r): ${err:-no error text}" \
              "run it by hand to see more: ssh -v $user@$host true" ;;
        esac
      fi
    fi
  elif [ -z "$target" ]; then
    _say skip "port 22 / ssh: no address for the Pi"
  else
    _say skip "ssh key auth: port 22 not reachable"
  fi

  # 7. The bridge, from the laptop -- over every address family we have --------------
  local bstate=none bmsg='' bfix='' fam
  if [ "$ssh_ok" = 1 ] || [ "$sshd" = yes ]; then
    for a in $v6 $v4; do
      c=$(_cap 3)
      if [ "$c" -lt 1 ]; then
        _say skip "bridge on $a: out of time"
        continue
      fi
      case $a in *:*) fam=IPv6 ;; *) fam=IPv4 ;; esac
      r=$(tcp_probe "$a" "$port" "$c")
      case $r in
        *'"type":"hello"'* | *'"type": "hello"'*)
          bstate=ok
          _say ok "bridge answers on $fam [$a]:$port (idle, ready for a laptop)"
          ;;
        *busy*)
          bstate=ok
          _say ok "bridge answers on $fam [$a]:$port but another laptop holds it (busy)"
          ;;
        OPEN*)
          [ "$bstate" = ok ] || bstate=other
          _say warn "something answers on [$a]:$port but it is not the bridge: ${r#OPEN }" \
            "pick a free port for the bridge (--port)"
          ;;
        REFUSED)
          if [ "$bstate" != ok ]; then bstate=down; fi
          if [ -n "$pi_listen" ]; then
            bmsg="bridge is running but not on $fam: it listens on $pi_listen only"
            bfix="restart it bound to all addresses: scripts/pi_deploy.sh --run"
            [ "$bstate" = ok ] || bstate=family
            _say warn "$bmsg" "$bfix"
          else
            _say info "bridge not listening on [$a]:$port"
          fi
          ;;
        *)
          if [ "$bstate" != ok ]; then bstate=unreach; fi
          _say warn "bridge port [$a]:$port: ${r} (firewall, or the link dropped)"
          ;;
      esac
    done
  else
    _say skip "bridge port: the Pi is not reachable"
  fi
  if [ "$bstate" = down ]; then
    bmsg="the bridge is not running"
    bfix="scripts/pi_deploy.sh --run"
    [ "$ssh_ok" = 1 ] && printf '%s\n' "$info" | grep -q '^pi_svc=active' &&
      bfix="the systemd service is active but not listening: journalctl -u retriever-bridge -n 30"
  elif [ "$bstate" != ok ] && [ -z "$bmsg" ]; then
    bmsg="the bridge is not answering on :$port"
    bfix="scripts/pi_deploy.sh --run"
  fi

  # Verdict -----------------------------------------------------------------------
  local took=$((SECONDS - D_T0)) via='over the network'
  if [ -n "$v4" ]; then
    via="over IPv4"
  elif [[ $v6 == [fF][eE]80:* ]]; then
    via="over IPv6 link-local"
  elif [ -n "$v6" ]; then
    via="over IPv6"
  fi
  echo
  local nm=''
  [ -n "$D_NM" ] && nm=" WARNING: eth0 will keep dropping -- NetworkManager waits for DHCP; $(nm_fix)."
  if [ "$ssh_ok" = 1 ] && [ "$bstate" = ok ]; then
    echo "VERDICT: OK -- ssh $user@$host works ($via) and the bridge answers on :$port.$nm (${took}s)"
    return 0
  elif [ "$ssh_ok" = 1 ]; then
    echo "VERDICT: Pi reachable (ssh $user@$host works, $via) but $bmsg. Fix: $bfix.$nm (${took}s)"
    return 3
  fi
  if [ -z "$D_FAIL" ]; then
    D_FAIL="ran out of time before reaching a conclusion"
    D_FIX="run it again with --budget 40"
  fi
  echo "VERDICT: CANNOT USE THE PI -- $D_FAIL.${D_FIX:+ Fix: $D_FIX} (${took}s)"
  return 1
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then
  doctor_main "$@"
  exit $?
fi
