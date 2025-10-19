#!/usr/bin/env python3
"""
Add language switcher buttons to all markdown pages in source and source_zh directories.
The buttons will be placed at the beginning of each file with relative path links.
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
    patterns = [
        r'\[.*?English.*?\]',
        r'\[.*?中文.*?\]',
        r'\[.*?EN.*?\]',
        r'\[.*?ZH.*?\]',
    ]
    for pattern in patterns:
        if re.search(pattern, content[:500]):  # Check first 500 chars
            return True
    return False

def add_language_switcher(file_path, is_chinese=False):
    """Add language switcher to a markdown file."""

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
            switcher = f"[English]({rel_path}) | **中文**\n\n---\n\n"
        else:
            print(f"  ⚠️  Warning: English version not found for {file_path}")
            return False
    else:
        # source -> source_zh
        zh_path = str(file_path).replace('/source/', '/source_zh/')
        if os.path.exists(zh_path):
            rel_path = get_relative_path(file_path, zh_path)
            switcher = f"**English** | [中文]({rel_path})\n\n---\n\n"
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

def process_directory(base_dir, is_chinese=False):
    """Process all markdown files in a directory."""
    print(f"\n{'='*60}")
    print(f"Processing {'Chinese' if is_chinese else 'English'} files in: {base_dir}")
    print(f"{'='*60}")

    count = 0
    for root, dirs, files in os.walk(base_dir):
        for file in files:
            if file.endswith('.md'):
                file_path = os.path.join(root, file)
                if add_language_switcher(file_path, is_chinese):
                    count += 1

    print(f"\n✨ Processed {count} files in {base_dir}")
    return count

def main():
    """Main function."""
    script_dir = Path(__file__).parent.resolve()

    source_dir = script_dir / 'source'
    source_zh_dir = script_dir / 'source_zh'

    print("🚀 Adding language switchers to all markdown pages...")
    print(f"📁 Base directory: {script_dir}")

    # Check if directories exist
    if not source_dir.exists():
        print(f"❌ Error: {source_dir} does not exist!")
        return

    if not source_zh_dir.exists():
        print(f"❌ Error: {source_zh_dir} does not exist!")
        return

    # Process English files
    en_count = process_directory(source_dir, is_chinese=False)

    # Process Chinese files
    zh_count = process_directory(source_zh_dir, is_chinese=True)

    print(f"\n{'='*60}")
    print(f"🎉 Done!")
    print(f"   Total English files processed: {en_count}")
    print(f"   Total Chinese files processed: {zh_count}")
    print(f"   Total files processed: {en_count + zh_count}")
    print(f"{'='*60}\n")

if __name__ == '__main__':
    main()
