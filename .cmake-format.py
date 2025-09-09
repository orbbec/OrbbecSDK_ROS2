# -----------------------------
# Options affecting formatting.
# -----------------------------
with section("format"):

  # Disable formatting entirely, making cmake-format a no-op
  disable = False

  # How wide to allow formatted cmake files
  line_width = 120

  # How many spaces to tab for indent
  tab_size = 4

  # If true, lines are indented using tab characters (utf-8 0x09) instead of
  # <tab_size> space characters (utf-8 0x20). In cases where the layout would
  # require a fractional tab character, the behavior of the  fractional
  # indentation is governed by <fractional_tab_policy>
  use_tabchars = False

  # If <use_tabchars> is True, then the value of this variable indicates how
  # fractional indentions are handled during whitespace replacement. If set to
  # 'use-space', fractional indentation is left as spaces (utf-8 0x20). If set
  # to `round-up` fractional indentation is replaced with a single tab character
  # (utf-8 0x09) effectively shifting the column to the next tabstop
  fractional_tab_policy = 'use-space'

  # If an argument group contains more than this many sub-groups (parg or kwarg
  # groups) then force it to a vertical layout.
  max_subgroups_hwrap = 4

  # If a positional argument group contains more than this many arguments, then
  # force it to a vertical layout.
  max_pargs_hwrap = 6

  # If a cmdline positional group consumes more than this many lines without
  # nesting, then invalidate the layout (and nest)
  max_rows_cmdline = 2

  # If true, separate flow control names from their parentheses with a space
  separate_ctrl_name_with_space = False

  # If true, separate function names from parentheses with a space
  separate_fn_name_with_space = False

  # If a statement is wrapped to more than one line, than dangle the closing
  # parenthesis on its own line.
  dangle_parens = True

  # If the trailing parenthesis must be 'dangled' on its on line, then align it
  # to this reference: `prefix`: the start of the statement,  `prefix-indent`:
  # the start of the statement, plus one indentation  level, `child`: align to
  # the column of the arguments
  dangle_align = 'prefix'

  # If the statement spelling length (including space and parenthesis) is
  # smaller than this amount, then force reject nested layouts.
  min_prefix_chars = 4

  # If the statement spelling length (including space and parenthesis) is larger
  # than the tab width by more than this amount, then force reject un-nested
  # layouts.
  max_prefix_chars = 10

  # If a candidate layout is wrapped horizontally but it exceeds this many
  # lines, then reject the layout.
  max_lines_hwrap = 2

  # What style line endings to use in the output.
  line_ending = 'unix'

  # Format command names consistently as 'lower' or 'upper' case
  command_case = 'canonical'

  # Format keywords consistently as 'lower' or 'upper' case
  keyword_case = 'unchanged'

  # A list of command names which should always be wrapped
  always_wrap = []

  # If true, the argument lists which are known to be sortable will be sorted
  # lexicographicall
  enable_sort = True

  # If true, the parsers may infer whether or not an argument list is sortable
  # (without annotation).
  autosort = False

  # By default, if cmake-format cannot successfully fit everything into the
  # desired linewidth it will apply the last, most agressive attempt that it
  # made. If this flag is True, however, cmake-format will print error, exit
  # with non-zero status code, and write-out nothing
  require_valid_layout = False

  # A dictionary mapping layout nodes to a list of wrap decisions. See the
  # documentation for more information.
  layout_passes = {}

# -----------------------------
# Options affecting comment formatting (indirectly).
# -----------------------------
with section("markup"):
    # Disabling markup avoids merging consecutive comment lines
    enable_markup = False