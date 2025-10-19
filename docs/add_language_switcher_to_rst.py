#!/usr/bin/env python3
"""
Add language switcher buttons to all RST chapter index files in source and source_zh directories.
"""

import os
import re
from pathlib import Path

def get_relative_path(from_path, to_path):
    """Calculate relative path from one file to another."""
    from_dir = os.path.dirname(from_path)
    rel_path = os.path.relpath(to_path, from_dir)
    return rel_path

def has_language_switcher(content):
    """Check if the file already has a language switcher."""
    # Check for common language switcher patterns in first few lines
    first_lines = '\n'.join(content.split('\n')[:10])
    patterns = [
        r'English.*中文',
        r'中文.*English',
        r'\*\*English\*\*',
        r'\*\*中文\*\*',
    ]
    for pattern in patterns:
        if re.search(pattern, first_lines, re.IGNORECASE):
            return True
    return False

def add_language_switcher_to_rst(file_path, is_chinese=False):
    """Add language switcher to an RST file."""

    # Read the file
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Check if already has switcher
    if has_language_switcher(content):
        print(f"  ⏭️  Skipping {file_path} (already has switcher)")
        return False

    # Get relative path to the corresponding file in the other language
    if is_chinese:
        # source_zh -> source
        en_path = str(file_path).replace('/source_zh/', '/source/')
        if os.path.exists(en_path):
            rel_path = get_relative_path(file_path, en_path)
            # For RST files, convert path to HTML
            rel_path_html = rel_path.replace('.rst', '.html')
            switcher = f"`English <{rel_path_html}>`_ | **中文**\n\n----\n\n"
        else:
            print(f"  ⚠️  Warning: English version not found for {file_path}")
            return False
    else:
        # source -> source_zh
        zh_path = str(file_path).replace('/source/', '/source_zh/')
        if os.path.exists(zh_path):
            rel_path = get_relative_path(file_path, zh_path)
            # For RST files, convert path to HTML
            rel_path_html = rel_path.replace('.rst', '.html')
            switcher = f"**English** | `中文 <{rel_path_html}>`_\n\n----\n\n"
        else:
            print(f"  ⚠️  Warning: Chinese version not found for {file_path}")
            return False

    # Add switcher at the beginning
    new_content = switcher + content

    # Write back
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(new_content)

    print(f"  ✅ Added switcher to {file_path}")
    return True

def process_rst_files(base_dir, is_chinese=False):
    """Process all RST chapter index files in a directory."""
    print(f"\n{'='*60}")
    print(f"Processing {'Chinese' if is_chinese else 'English'} RST files in: {base_dir}")
    print(f"{'='*60}")

    count = 0

    # List of RST chapter index files to process
    rst_files = [
        '1_overview/overview.rst',
        '2_installation/installation.rst',
        '3_quickstarts/quickstarts.rst',
        '4_application_guide/application_guide.rst',
        '5_advanced_guide/advanced_guide.rst',
        '6_benchmark/benchmark.rst',
        '7_developer_guide/developer_guide.rst',
        '8_FAQ/FAQ.rst',
    ]

    for rst_file in rst_files:
        file_path = os.path.join(base_dir, rst_file)
        if os.path.exists(file_path):
            if add_language_switcher_to_rst(file_path, is_chinese):
                count += 1
        else:
            print(f"  ⚠️  File not found: {file_path}")

    print(f"\n✨ Processed {count} RST files in {base_dir}")
    return count

def main():
    """Main function."""
    script_dir = Path(__file__).parent.resolve()

    source_dir = script_dir / 'source'
    source_zh_dir = script_dir / 'source_zh'

    print("🚀 Adding language switchers to RST chapter index files...")
    print(f"📁 Base directory: {script_dir}")

    # Check if directories exist
    if not source_dir.exists():
        print(f"❌ Error: {source_dir} does not exist!")
        return

    if not source_zh_dir.exists():
        print(f"❌ Error: {source_zh_dir} does not exist!")
        return

    # Process English RST files
    en_count = process_rst_files(source_dir, is_chinese=False)

    # Process Chinese RST files
    zh_count = process_rst_files(source_zh_dir, is_chinese=True)

    print(f"\n{'='*60}")
    print(f"🎉 Done!")
    print(f"   Total English RST files processed: {en_count}")
    print(f"   Total Chinese RST files processed: {zh_count}")
    print(f"   Total RST files processed: {en_count + zh_count}")
    print(f"{'='*60}\n")

if __name__ == '__main__':
    main()
