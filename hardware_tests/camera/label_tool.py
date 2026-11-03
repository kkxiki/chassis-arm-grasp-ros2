#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 标定标注工具（无第三方依赖）：显示每组原图，用户逐张点击 TCP 尖端位置
# 用法: python label_tool.py --data <pairs_all目录> --out labels.json --port 8091
import argparse
import json
import os
from http.server import BaseHTTPRequestHandler, HTTPServer


def list_groups(dirpath):
    groups = []
    for n in sorted(os.listdir(dirpath)):
        if not n.isdigit():
            continue
        d = os.path.join(dirpath, n)
        if os.path.exists(os.path.join(d, "color.png")) and \
           os.path.exists(os.path.join(d, "board.json")):
            groups.append(n)
    return groups


class Handler(BaseHTTPRequestHandler):
    data_dir = None
    outpath = None
    labels = {}
    groups = []

    def _send(self, code, body, ctype="application/octet-stream"):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/":
            self._send(200, self._page().encode("utf-8"), "text/html; charset=utf-8")
        elif self.path.startswith("/img/"):
            n = self.path.split("/")[-1]
            p = os.path.join(self.data_dir, n, "color.png")
            if n in self.groups and os.path.exists(p):
                with open(p, "rb") as f:
                    self._send(200, f.read(), "image/png")
            else:
                self._send(404, b"not found")
        elif self.path == "/api/list":
            self._send(200, json.dumps({"names": self.groups,
                                        "marked": {k: v for k, v in self.labels.items()}}).encode())
        else:
            self._send(404, b"not found")

    def do_POST(self):
        if self.path == "/api/save":
            ln = int(self.headers.get("Content-Length", 0))
            data = json.loads(self.rfile.read(ln))
            self.labels = data.get("labels", {})
            with open(self.outpath, "w") as f:
                json.dump({"labels": self.labels}, f, indent=2)
            self._send(200, json.dumps({"ok": True, "count": len(self.labels)}).encode())
        else:
            self._send(404, b"not found")

    def log_message(self, *a):
        pass

    @staticmethod
    def _page():
        return """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>标定标注工具</title>
<style>
body{font-family:Consolas,monospace;background:#111;color:#eee;margin:0;padding:12px}
#wrap{max-width:1100px;margin:auto}
h3{margin:6px 0}
#imgbox{position:relative;display:inline-block}
img{display:block;max-width:100%;cursor:crosshair}
#mark{position:absolute;width:16px;height:16px;border:3px solid red;border-radius:50%;transform:translate(-50%,-50%);pointer-events:none;display:none}
#info{font-size:14px;color:#aaa;margin:6px 0}
button{font-size:15px;padding:6px 14px;margin:4px;background:#2a6;color:#fff;border:none;border-radius:4px;cursor:pointer}
button:hover{background:#3a7}
#status{color:#8f8}
.note{color:#f80;font-size:13px}
#coord{font-size:14px;color:#ff6}
</style></head><body><div id="wrap">
<h3>标定标注工具 — 点击每组图中 <span style="color:#f66">TCP 尖端</span> 的位置</h3>
<p class="note">提示：尖端 = 机械臂末端最尖处（触针/笔尖）。每张图点一下尖端位置，会弹出对话框确认尖端是否接触棋盘表面（接触=确定 / 悬空=取消）。</p>
<div id="info"></div>
<div id="imgbox"><img id="img" alt=""><div id="mark"></div></div><br>
<button onclick="prev()">◀ 上一张</button>
<button onclick="next()">下一张 ▶</button>
<button onclick="undo()">撤销本张</button>
<button onclick="save()">💾 保存</button>
<span id="status"></span>
<p id="coord"></p>
<script>
let names=[], idx=0;
const labels={};
const mark=document.getElementById('mark');
const img=document.getElementById('img');
async function load(){
  const r=await fetch('/api/list'); const d=await r.json();
  names=d.names; for(const [k,v] of Object.entries(d.marked)) labels[k]=v;
  render();
}
function render(){
  if(names.length===0)return;
  const n=names[idx];
  img.src='/img/'+n;
  const cur=labels[n];
  if(cur){mark.style.display='block';} else {mark.style.display='none';}
  document.getElementById('info').innerText=
    '图像 '+(idx+1)+'/'+names.length+'  (组 '+n+')   [已标注 '+Object.keys(labels).length+' 张]';
}
img.addEventListener('click',e=>{
  const rect=img.getBoundingClientRect();
  const u=(e.clientX-rect.left)*(img.naturalWidth/rect.width);
  const v=(e.clientY-rect.top)*(img.naturalHeight/rect.height);
  const n=names[idx];
  const touch=window.confirm('尖端是否接触棋盘表面？\\n确定 = 接触（推荐）  取消 = 悬空');
  labels[n]={u:Math.round(u),v:Math.round(v),touch:touch};
  mark.style.left=(e.clientX-rect.left)+'px';
  mark.style.top=(e.clientY-rect.top)+'px';
  mark.style.display='block';
  document.getElementById('coord').innerText=n+' -> ('+Math.round(u)+','+Math.round(v)+')  接触:'+(touch?'是':'否(悬空)');
});
function prev(){if(idx>0){idx--;render();}}
function next(){if(idx<names.length-1){idx++;render();}else{alert('已是最后一张');}}
function undo(){const n=names[idx];delete labels[n];mark.style.display='none';document.getElementById('coord').innerText='';render();}
async function save(){
  const r=await fetch('/api/save',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({labels})});
  const d=await r.json();
  document.getElementById('status').innerText=' 已保存 '+d.count+' 组';
}
load();
</script></body></html>"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=r"e:\XuK\底盘机械臂【0819】\hardware_tests\camera\pairs_all")
    ap.add_argument("--out", default=r"e:\XuK\底盘机械臂【0819】\hardware_tests\camera\pairs_all\labels.json")
    ap.add_argument("--port", type=int, default=8091)
    a = ap.parse_args()
    Handler.data_dir = a.data
    Handler.outpath = a.out
    Handler.groups = list_groups(a.data)
    if os.path.exists(a.out):
        Handler.labels = json.load(open(a.out)).get("labels", {})
    srv = HTTPServer(("127.0.0.1", a.port), Handler)
    print("标注工具已启动: http://127.0.0.1:%d   (%d 张图像)" % (a.port, len(Handler.groups)))
    print("已载入已有标注: %d 组" % len(Handler.labels))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
