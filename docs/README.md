# OrbbecSDK V2 ROS2 Documentation

This repository contains the documentation for OrbbecSDK V2 ROS2 Wrapper in both English and Chinese.

## Documentation Structure

- **English Version**: `docs/en/` - Full English documentation
- **中文版本**: `docs/zh/` - 完整中文文档
- **Published Versions**: `docs/version-manifest.json` - Version list for GitHub Pages

## Quick Start

### Build English Documentation:
```bash
cd docs/en
pip install -r requirements.txt
make clean && make html
```

### Build Chinese Documentation:
```bash
cd docs/zh
pip install -r requirements.txt
make clean && make html
```

### For Full Versioned Site Build:
```bash
pip install -r docs/en/requirements.txt -r docs/zh/requirements.txt jieba
python docs/build_versions.py
```

## View Documentation

After building, open the following files in your browser:
- English: `docs/en/_build/html/index.html`
- Chinese: `docs/zh/_build/html/index.html`
- Full GitHub Pages site: `site/index.html`

## Project Structure

```
docs/
├── en/                      # English documentation
│   ├── source/             # Source files
│   ├── conf.py             # Sphinx config
│   └── requirements.txt    # Python dependencies
├── zh/                      # Chinese documentation
│   ├── source/             # Source files
│   ├── conf.py             # Sphinx config
│   └── requirements.txt    # Python dependencies
├── _ext/                    # Shared Sphinx extensions
├── build_versions.py        # Versioned site builder
└── version-manifest.json    # Published version definitions
```

## Notes

1. Make changes to the English version first (`docs/en/`)
2. Translate and adapt for the Chinese version (`docs/zh/`)
3. Ensure both versions build successfully

## Example Documentation

For examples documented on Pages, the current-version guide is the source of truth for complete usage, configuration, and troubleshooting. Keep their READMEs in the code repository to a short purpose, prerequisites or local settings, run command, and links to the English and Chinese guides. Add new guides to both language navigation trees before shortening an existing README. Keep the example index linked to the source directory and guides.

The Gemini 435Le example is documented in its source-directory README only; keep its full instructions there and link directly to that README from the example index.

## Dependencies

- Sphinx
- recommonmark
- sphinx-rtd-theme
- jieba (for Chinese search)
- Required extensions (see requirements.txt in each language folder)
