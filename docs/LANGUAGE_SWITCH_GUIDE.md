# 双语文档语言切换指南

## GitHub Pages 部署结构

当推送到 GitHub 后，文档将自动构建并部署到以下结构：

```
https://orbbec.github.io/OrbbecSDK_ROS2/
├── index.html              # 英文版首页（默认）
├── _static/                # 英文版静态资源
├── 1_overview/             # 英文版内容
├── 2_installation/
├── ...
└── zh/                     # 中文版
    ├── index.html          # 中文版首页
    ├── _static/            # 中文版静态资源
    ├── 1_overview/         # 中文版内容
    └── ...
```

## 访问地址

- **英文版（默认）**: `https://orbbec.github.io/OrbbecSDK_ROS2/`
- **中文版**: `https://orbbec.github.io/OrbbecSDK_ROS2/zh/`

## 如何添加语言切换链接

### 方法 1：在自定义 CSS 中添加语言切换按钮（推荐）

编辑 `docs/en/source/_static/custom.css` 和 `docs/zh/source/_static/custom.css`：

```css
/* 在页面右上角添加语言切换链接 */
.wy-nav-top::after {
    content: "中文";
    position: absolute;
    right: 60px;
    top: 0;
    height: 45px;
    line-height: 45px;
    padding: 0 15px;
    font-size: 14px;
    color: #fff;
    cursor: pointer;
}

/* 为链接添加点击事件 */
.wy-nav-top {
    position: relative;
}
```

### 方法 2：在自定义 JavaScript 中添加（更灵活）

创建或编辑 `docs/en/source/_static/language-switch.js`：

```javascript
document.addEventListener('DOMContentLoaded', function() {
    // 创建语言切换链接
    var langSwitch = document.createElement('a');
    langSwitch.href = '/OrbbecSDK_ROS2/zh/';
    langSwitch.textContent = '中文';
    langSwitch.style.cssText = 'position: fixed; top: 10px; right: 60px; z-index: 1000; color: #fff; background: #2980b9; padding: 5px 15px; border-radius: 3px; text-decoration: none;';

    document.body.appendChild(langSwitch);
});
```

然后在 `docs/en/conf.py` 中添加：

```python
html_js_files = [
    'language-switch.js',
]
```

对应的中文版 `docs/zh/source/_static/language-switch.js`：

```javascript
document.addEventListener('DOMContentLoaded', function() {
    var langSwitch = document.createElement('a');
    langSwitch.href = '/OrbbecSDK_ROS2/';
    langSwitch.textContent = 'English';
    langSwitch.style.cssText = 'position: fixed; top: 10px; right: 60px; z-index: 1000; color: #fff; background: #2980b9; padding: 5px 15px; border-radius: 3px; text-decoration: none;';

    document.body.appendChild(langSwitch);
});
```

### 方法 3：修改 Sphinx 主题模板（最专业）

1. 创建自定义模板覆盖 `docs/en/source/_templates/layout.html`：

```html
{% extends "!layout.html" %}

{% block extrahead %}
  {{ super() }}
  <style>
    .language-switch {
      position: fixed;
      top: 10px;
      right: 60px;
      z-index: 1000;
    }
    .language-switch a {
      color: #fff;
      background: #2980b9;
      padding: 5px 15px;
      border-radius: 3px;
      text-decoration: none;
      font-size: 14px;
    }
    .language-switch a:hover {
      background: #3498db;
    }
  </style>
{% endblock %}

{% block menu %}
  {{ super() }}
  <div class="language-switch">
    <a href="/OrbbecSDK_ROS2/zh/">中文</a>
  </div>
{% endblock %}
```

2. 对应的中文版 `docs/zh/source/_templates/layout.html`：

```html
{% extends "!layout.html" %}

{% block extrahead %}
  {{ super() }}
  <style>
    .language-switch {
      position: fixed;
      top: 10px;
      right: 60px;
      z-index: 1000;
    }
    .language-switch a {
      color: #fff;
      background: #2980b9;
      padding: 5px 15px;
      border-radius: 3px;
      text-decoration: none;
      font-size: 14px;
    }
    .language-switch a:hover {
      background: #3498db;
    }
  </style>
{% endblock %}

{% block menu %}
  {{ super() }}
  <div class="language-switch">
    <a href="/OrbbecSDK_ROS2/">English</a>
  </div>
{% endblock %}
```

## 在 conf.py 中配置主题选项（可选）

你也可以在 `conf.py` 中配置 RTD 主题的自定义选项：

```python
html_theme_options = {
    # ...existing options...
    'display_version': True,
    'prev_next_buttons_location': 'bottom',
    'style_external_links': False,
    # 可以添加自定义 HTML
    # 'canonical_url': '',
}
```

## 本地测试

在推送到 GitHub 之前，建议先本地测试：

```bash
# 构建英文版
cd docs/en
make html
# 在浏览器中打开：docs/en/_build/html/index.html

# 构建中文版
cd docs/zh
make html
# 在浏览器中打开：docs/zh/_build/html/index.html
```

## GitHub Actions 工作流程

当你推送代码到 `Document` 分支时，GitHub Actions 会自动：

1. ✅ 安装 Python 和依赖
2. ✅ 构建英文版文档到 `docs/en/_build/html/`
3. ✅ 构建中文版文档到 `docs/zh/_build/html/`
4. ✅ 将英文版作为根目录
5. ✅ 将中文版复制到 `/zh/` 子目录
6. ✅ 部署到 `gh-pages` 分支

## 推荐的实现方式

我推荐使用 **方法 2（JavaScript）** 或 **方法 3（模板）**，因为：

- ✅ 更灵活，易于维护
- ✅ 可以动态调整样式
- ✅ 不依赖于特定主题的 CSS 类名
- ✅ 可以添加更多交互功能（如自动检测浏览器语言）

## 注意事项

1. **链接路径**：注意区分相对路径和绝对路径
   - 英文版链接到中文版：`/OrbbecSDK_ROS2/zh/`
   - 中文版链接到英文版：`/OrbbecSDK_ROS2/`

2. **测试链接**：在部署后测试所有语言切换链接是否正常工作

3. **SEO 优化**：可以在 HTML head 中添加 `hreflang` 标签：
   ```html
   <link rel="alternate" hreflang="en" href="/OrbbecSDK_ROS2/" />
   <link rel="alternate" hreflang="zh-CN" href="/OrbbecSDK_ROS2/zh/" />
   ```
