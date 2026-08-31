# 归档 — 开发/测试工具

开发过程中用到的测试脚本，与游戏运行无关，归档于此备用。

| 文件 | 用途 |
|---|---|
| `测试-冒烟启动.js` | 在模拟浏览器（jsdom）里启动游戏，验证游戏数据/脚本没损坏。改过 `launcher.py` 或数据后跑一遍最稳 |
| `测试-验证配置.js` | 请求服务器生成的 `data.js`，核对各配置项（子弹冲量/寿命阈值/道具刷新）是否按 config.json 生效 |

## 使用前提

冒烟测试需要 Node.js 和 jsdom：

```bat
npm install jsdom --prefix "%TEMP%\tt3jsdom"
set JSDOM_PATH=%TEMP%\tt3jsdom
cd 游戏根目录
node 归档\测试-冒烟启动.js
```

验证配置需要游戏服务器在运行：

```bat
node 归档\测试-验证配置.js http://127.0.0.1:端口
```

## 说明

- 两个脚本都只读不改任何游戏文件，放心使用。
- 冒烟测试通过标准：输出 `runtime: created` 且 `errors (0)`。
- 若 jsdom 安装失败，多半是 npm 缓存目录权限问题，可加 `--cache <可写目录>` 重试。
