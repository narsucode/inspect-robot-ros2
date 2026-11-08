#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把巡检 JSON 报告渲染成一张可视化网页，方便录屏或放进论文。

用法：
    python3 ~/narsu/ros2/tools/make_report_html.py
    python3 ~/narsu/ros2/tools/make_report_html.py patrol_20261108_111525.json
    python3 ~/narsu/ros2/tools/make_report_html.py --open

输出：
    ~/patrol_reports/report_<时间戳>.html
    截图以 base64 内嵌，单个文件即可拷走
"""

import base64
import json
import os
import sys
import time

REPORTS = os.path.expanduser('~/patrol_reports')
SHOTS = os.path.expanduser('~/patrol_shots')

# 颜色 → (中文, 圆点色, 背景色)
LEVEL_STYLE = {
    '正常': ('正常', '#2e9e4f', '#eaf7ee'),
    '警告': ('警告', '#d48806', '#fff7e6'),
    '故障': ('故障', '#d9363e', '#fff1f0'),
    '异常': ('异常', '#8c8c8c', '#f5f5f5'),
    '熄灭': ('熄灭', '#8c8c8c', '#f5f5f5'),
    '未知': ('未知', '#8c8c8c', '#f5f5f5'),
}

STATE_CN = {
    'GREEN': '绿灯', 'YELLOW': '黄灯', 'RED': '红灯',
    'OFF': '熄灭', 'UNKNOWN': '未识别', 'MULTI': '多灯',
    'NAV_FAILED': '导航失败', 'ARRIVED': '已到达', 'CAPTURED': '已拍照',
}


def img_b64(path):
    """把截图转成 data URI，HTML 单文件就能带走。"""
    if not path:
        return None
    p = path if os.path.isabs(path) else os.path.join(SHOTS, os.path.basename(path))
    if not os.path.exists(p):
        return None
    with open(p, 'rb') as f:
        b = base64.b64encode(f.read()).decode('ascii')
    return 'data:image/jpeg;base64,' + b


def build(data, out_path):
    results = data.get('results', [])
    stamp = data.get('time', '')
    tidy = stamp.replace('_', ' ')
    if len(tidy) >= 13:
        tidy = '%s-%s-%s %s:%s' % (tidy[0:4], tidy[4:6], tidy[6:8], tidy[9:11], tidy[11:13])

    n_ok = sum(1 for r in results if r.get('ok'))
    n_bad = len(results) - n_ok
    n_warn = sum(1 for r in results if r.get('level') == '警告')
    n_fail = sum(1 for r in results if r.get('level') == '故障')
    n_good = sum(1 for r in results if r.get('level') == '正常')

    rows = []
    for r in results:
        level = r.get('level') or ''
        state = r.get('state') or ''
        cn, dot, bg = LEVEL_STYLE.get(level, ('未知', '#8c8c8c', '#f5f5f5'))
        votes = r.get('votes') or {}
        vote_txt = ' / '.join('%s %d' % (STATE_CN.get(k, k), v) for k, v in votes.items()) or '-'
        shot = img_b64(r.get('image'))
        thumb = ('<img class="shot" src="%s" alt="截图">' % shot) if shot else '<span class="na">无截图</span>'
        rows.append(f"""
        <tr style="background:{bg}">
          <td class="point">{r.get('point','-')}</td>
          <td><span class="dot" style="background:{dot}"></span><b style="color:{dot}">{cn}</b></td>
          <td>{STATE_CN.get(state, state) or '-'}</td>
          <td class="num">{votes.get(state, r.get('hits', 0))}</td>
          <td class="vote">{vote_txt}</td>
          <td>{r.get('time','-')}</td>
          <td>{thumb}</td>
        </tr>""")

    back = data.get('back_ok')
    back_txt = '' if back is None else ('　返航：成功' if back else '　返航：未完成')
    generated = time.strftime('%Y-%m-%d %H:%M:%S')

    html = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>机房巡检报告 {tidy}</title>
<style>
  * {{ box-sizing: border-box; }}
  body {{
    margin: 0; padding: 36px 44px;
    font-family: "Microsoft YaHei", "Noto Sans CJK SC", sans-serif;
    background: #fafafa; color: #222;
  }}
  h1 {{ margin: 0 0 6px; font-size: 30px; letter-spacing: 1px; }}
  .sub {{ color: #888; font-size: 15px; margin-bottom: 26px; }}
  .cards {{ display: flex; gap: 16px; margin-bottom: 28px; }}
  .card {{
    flex: 1; background: #fff; border-radius: 10px; padding: 18px 22px;
    box-shadow: 0 1px 3px rgba(0,0,0,.08);
  }}
  .card .k {{ color: #888; font-size: 14px; margin-bottom: 8px; }}
  .card .v {{ font-size: 34px; font-weight: 700; line-height: 1; }}
  .card.ok .v {{ color: #2e9e4f; }}
  .card.warn .v {{ color: #d48806; }}
  .card.bad .v {{ color: #d9363e; }}
  table {{ width: 100%; border-collapse: collapse; background: #fff; border-radius: 10px; overflow: hidden; box-shadow: 0 1px 3px rgba(0,0,0,.08); }}
  th {{ background: #f0f0f0; font-size: 15px; padding: 13px 12px; text-align: left; color: #555; font-weight: 600; }}
  td {{ padding: 12px; font-size: 16px; border-top: 1px solid #eee; vertical-align: middle; }}
  td.point {{ font-size: 20px; font-weight: 700; }}
  td.num {{ font-size: 18px; font-weight: 600; }}
  td.vote {{ color: #666; font-size: 14px; }}
  .dot {{ display: inline-block; width: 14px; height: 14px; border-radius: 50%; margin-right: 8px; vertical-align: -2px; }}
  .shot {{ height: 68px; border-radius: 6px; border: 1px solid #ddd; }}
  .na {{ color: #bbb; font-size: 14px; }}
  .foot {{ margin-top: 20px; color: #999; font-size: 13px; }}
</style>
</head>
<body>
  <h1>机房设备巡检报告</h1>
  <div class="sub">巡检时间 {tidy}　·　共 {len(results)} 个点位{back_txt}</div>

  <div class="cards">
    <div class="card ok"><div class="k">正常</div><div class="v">{n_good}</div></div>
    <div class="card warn"><div class="k">警告</div><div class="v">{n_warn}</div></div>
    <div class="card bad"><div class="k">故障</div><div class="v">{n_fail}</div></div>
    <div class="card"><div class="k">合计 / 异常</div><div class="v">{len(results)} / {n_bad}</div></div>
  </div>

  <table>
    <thead>
      <tr><th>点位</th><th>判定</th><th>识别结果</th><th>票数</th><th>投票明细</th><th>时间</th><th>现场截图</th></tr>
    </thead>
    <tbody>{''.join(rows)}
    </tbody>
  </table>

  <div class="foot">生成于 {generated}　·　数据来源 {os.path.basename(out_path).replace('.html','.json')}</div>
</body>
</html>"""

    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(html)
    return out_path


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    do_open = '--open' in sys.argv

    if args:
        src = args[0]
        if not os.path.isabs(src):
            src = os.path.join(REPORTS, src)
    else:
        cands = [os.path.join(REPORTS, f) for f in os.listdir(REPORTS)
                 if f.startswith('patrol_') and f.endswith('.json')]
        if not cands:
            print('没有找到 patrol_*.json')
            return
        # 跳过 0 字节的空报告
        cands = [c for c in cands if os.path.getsize(c) > 10]
        src = max(cands, key=os.path.getmtime)

    with open(src, encoding='utf-8') as f:
        data = json.load(f)

    stem = os.path.splitext(os.path.basename(src))[0]
    out = os.path.join(REPORTS, 'report_%s.html' % stem.replace('patrol_', ''))
    build(data, out)
    print('已生成: %s' % out)

    if do_open:
        os.system('xdg-open "%s" >/dev/null 2>&1 &' % out)


if __name__ == '__main__':
    main()
