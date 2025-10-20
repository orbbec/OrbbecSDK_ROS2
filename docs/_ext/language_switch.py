"""Sphinx extension to inject language switch links between English (en) and Chinese (zh) builds.
Updated to work with GitHub Pages project URLs like /<repo>/en/... and /<repo>/zh/....
"""
from docutils import nodes


def setup(app):
    app.connect('doctree-resolved', inject_language_switch)
    return {"version": "0.4", "parallel_read_safe": True}


def inject_language_switch(app, doctree, docname):
    lang = app.config.language or 'en'
    is_en = lang.startswith('en')
    other_label = '中文' if is_en else 'English'
    current_label = 'English' if is_en else '中文'

    # JS now detects /en/ or /zh/ segment and swaps only the first occurrence
    html = (
        '<div class="lang-switch" style="margin:0 0 1em 0; font-size:0.9em;">'
        f'<a class="lang-other" href="#">{other_label}</a> | '
        f'<a class="lang-current" href="#">{current_label}</a>'
        '</div>'
        '<script>(function(){'
        "var p=window.location.pathname;"
        "var isEn=p.indexOf('/en/')!==-1 && p.indexOf('/zh/')===-1;"
        "if(p.indexOf('/zh/')!==-1) isEn=false;"
        "var other = isEn ? p.replace(/\\/en\\//, '/zh/') : p.replace(/\\/zh\\//, '/en/');"
        "var current=p;"
        "var sw=document.querySelector('.lang-switch'); if(!sw) return;"
        "var o=sw.querySelector('.lang-other'); var c=sw.querySelector('.lang-current');"
        "if(isEn){ o.textContent='中文'; c.textContent='English'; } else { o.textContent='English'; c.textContent='中文'; }"
        "o.setAttribute('href', other); c.setAttribute('href', current);"
        '})();</script>'
    )
    doctree.insert(0, nodes.raw('', html, format='html'))
