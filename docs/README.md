# Docs应用文档

**编译：**

```bash
cd docs
./autobuild.sh all

```

**打开文档：**

```bash
cd docs
./autobuild.sh run

```

或

进入_html目录，手动点击index.html打开

# Docs应用文档详细说明

**采用方式：**

1、文档格式 markdown

2、文档转换工具 Sphinx

3、托管 github


# gitlab：

本项目源码地址： https://code.orbbec.com.cn/OrbbecSDK/OrbbecSDK_ROS2.git

# github （待上线）

本项目源码地址 ：  待上线
本应用文档阅读链接 ： 待上线


# 文档编写规范

1、框图、软件框架建议原则上采用processon、visio工具绘图，目的是为了保持风格统一（特殊图除外）

2、采用标准markdown格式，可使用vscode的Editor插件编辑


# 编译环境

pip3 install sphinx

pip3 install recommonmark

pip3 install sphinx_markdown_tables

pip3 install sphinx_rtd_theme

pip3 install sphinx_book_theme

# 编译

- 执行编译脚本 ./autobuild 编译（推荐，该脚本编译后会打包docs文档及渲染设置）。 也支持sphinx原生编译指令make html
- 打开docs/index.html查看效果


# 提交

git add 、git commit 添加注释、git push提交代码

提交完后，务必要上线网页打开检查一下，检查网页效果无误


# 定制化部分

说明：定制化的配置都在**conf.py**中设置。也需要安装一些支持库

1. 更改样式主题。我这里以 `sphinx_rtd_theme`为例子，其他主题可自行百度。

* 安装 `sphinx_rtd_theme`:    ` pip install sphinx_rtd_theme`

2. 安装markdown语法支持插件：`pip install myst-parser`
3. 安装支持mermaid渲染插件: `pip install sphinxcontrib.mermaid`
4. 安装代码块一键复制按钮插件：`pip install sphinx_copybutton`
5. 可以使用vscode 编辑markdown文件文档，推荐安装markdown预览编辑插件
