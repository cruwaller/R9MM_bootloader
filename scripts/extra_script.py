Import("env")
#env.Append(LINKFLAGS=["-nostartfiles"])
env['LINKFLAGS'].remove("-nostartfiles")

env.ProcessUnFlags("-nostartfiles")