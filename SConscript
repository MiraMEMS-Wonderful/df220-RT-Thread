from building import *
Import('rtconfig')

src   = []
cwd   = GetCurrentDir()

# add df220 src files.
src += Glob('sensor_mira_df220.c')
src += Glob('libraries/df220.c')

# add df220 include path.
path  = [cwd, cwd + '/libraries']

# add src and include to group.
group = DefineGroup('df220', src, depend = ['PKG_USING_DF220'], CPPPATH = path)

Return('group')
