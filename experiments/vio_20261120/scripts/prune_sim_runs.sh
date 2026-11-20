#!/usr/bin/env bash
# [SIM-ONLY] 仿真日志轮次收尾：只保留最近 N 轮 stereo5m_<YYYYMMDD_HHMM> 目录。
# 只作用于本机 WSL ~/drone_sim/logs；不触碰 Nano、不触碰 GitHub、不进实机配置。
#
# 用法：
#   bash prune_sim_runs.sh                       # dry-run：只列清单，不删
#   bash prune_sim_runs.sh --apply              # 实际删除
#   KEEP_RUNS=5 bash prune_sim_runs.sh --apply  # 自定义保留轮数
#
# 由 run_stereo_5m.sh 的 cleanup() 在每轮收尾自动调用，输出追加到 logs/_prune.log。
# 为什么需要：每轮 px4_gz.log 约 700 MB，多轮累积会线性吃满磁盘。
set -uo pipefail

LOGROOT=/home/gengyinghao/drone_sim/logs
KEEP=${KEEP_RUNS:-3}
APPLY=0
for a in "$@"; do
  case "$a" in --apply) APPLY=1 ;; esac
done

cd "$LOGROOT" || { echo "logs root missing: $LOGROOT"; exit 1; }

# 严格命名匹配 + 只认目录；按目录名（含时间戳）升序，最新在最后。
runs=()
while IFS= read -r d; do
  [[ -d "$d" ]] || continue
  [[ "$d" =~ ^stereo5m_[0-9]{8}_[0-9]{4}$ ]] || continue
  runs+=("$d")
done < <(ls -1 | sort)

n=${#runs[@]}
if (( APPLY == 1 )); then MODE=APPLY; else MODE=DRY-RUN; fi
echo "[$(date '+%F %T')] 轮次总数=$n 保留最近=$KEEP 模式=$MODE"

if (( n <= KEEP )); then
  echo "无需删除（n<=KEEP）"
  echo "logs 总量: $(du -sh . 2>/dev/null | cut -f1)"
  exit 0
fi

cut=$((n-KEEP))
to_del=("${runs[@]:0:cut}")
keep=("${runs[@]:cut}")

echo "=== 保留 ==="
printf '  %s\n' "${keep[@]}"
echo "=== 待删（$cut 轮）==="
printf '  %s\n' "${to_del[@]}"

# 二次保护：当前正在写入的轮次绝不删（外部设置 SIM_RUN 时生效）。
self_name=""
if [[ -n "${SIM_RUN:-}" ]]; then self_name=$(basename "$SIM_RUN"); fi

freed=0; ok=0; fail=0
for d in "${to_del[@]}"; do
  [[ -d "$d" ]] || continue
  [[ "$d" =~ ^stereo5m_[0-9]{8}_[0-9]{4}$ ]] || { echo "SKIP 名称不合规: $d"; continue; }
  if [[ "$d" == "$self_name" ]]; then echo "SKIP 当前轮: $d"; continue; fi
  sz=$(du -sm "$d" 2>/dev/null | cut -f1)
  [[ -z "$sz" ]] && sz=0
  if (( APPLY == 1 )); then
    if rm -rf -- "$d"; then
      ok=$((ok+1)); freed=$((freed+sz))
    else
      fail=$((fail+1)); echo "FAILED: $d"
    fi
  else
    freed=$((freed+sz))
  fi
done

echo "=== 结果 ==="
echo "删除轮次=$ok 失败=$fail 释放约=${freed}MB"
echo "logs 总量: $(du -sh . 2>/dev/null | cut -f1)"
