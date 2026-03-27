AddOption('--minimal',
          action='store_false',
          dest='extras',
          default=True,
          help='the minimum build. no tests, tools, etc.')

AddOption('--ubsan',
          action='store_true',
          help='turn on UBSan')

import os
env = Environment(
  ENV=os.environ,
  COMPILATIONDB_USE_ABSPATH=True,
  tools=["default", "compilation_db"],
)

env.CompilationDatabase("compile_commands.json")

# panda fw & test files
SConscript('SConscript')
