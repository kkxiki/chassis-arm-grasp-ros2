#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 分析控制器前端 JS：/nd-mgr/log 请求参数、token、登录接口
import re

s = open('/tmp/web.js', encoding='utf-8', errors='replace').read()

print('==== nd-mgr/log 调用上下文（每个匹配前后 300 字符）====')
for m in re.finditer(r'nd-mgr/log', s):
    seg = s[max(0, m.start()-300):m.end()+300]
    print(seg.replace('\n', ' '))
    print('======')

print('\n==== user-login / 登录相关 ====')
for m in re.finditer(r'auth-service[^"\'`,;]{0,60}', s):
    print(m.group(0))
    print('---')
