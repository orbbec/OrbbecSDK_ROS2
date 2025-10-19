"""Sphinx extension to inject language switch links between English (en) and Chinese (zh) builds.
Works for local file:// viewing: replaces the segment /en/_build/html/ with /zh/_build/html/ and vice versa.
Removes need for hard-coded absolute paths.
"""
from docutils import nodes


def setup(app):
    app.connect('doctree-resolved', inject_language_switch)
    return {"version": "0.3", "parallel_read_safe": True}


def inject_language_switch(app, doctree, docname):
    lang = app.config.language or 'en'
    is_en = lang.startswith('en')
    # Placeholder labels; JS will adjust depending on actual page path
    other_label = '中文' if is_en else 'English'
    current_label = 'English' if is_en else '中文'

    html = (
        '<div class="lang-switch" style="margin:0 0 1em 0; font-size:0.9em;">'
        f'<a class="lang-other" href="#">{other_label}</a> | '
        f'<a class="lang-current" href="#">{current_label}</a>'
        '</div>'
        '<script>(function(){'
        "var p=window.location.pathname;"
        "var isEn=p.indexOf('/en/_build/html/')!==-1;"
        "var other=isEn? p.replace('/en/_build/html/','/zh/_build/html/') : p.replace('/zh/_build/html/','/en/_build/html/');"
        "var current=p;"
        "var sw=document.querySelector('.lang-switch'); if(!sw) return;"
        "var o=sw.querySelector('.lang-other'); var c=sw.querySelector('.lang-current');"
        "if(isEn){ o.textContent='中文'; c.textContent='English'; } else { o.textContent='English'; c.textContent='中文'; }"
        "o.setAttribute('href', other); c.setAttribute('href', current);"
        '})();</script>'
    )
    doctree.insert(0, nodes.raw('', html, format='html'))
