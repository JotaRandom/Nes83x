project('nes83x', 'c',
  version : '0.0.1',
  default_options : [
    'warning_level=3',
    'c_std=c11',
    'default_library=static'
  ]
)

# Dependencies
gtk_dep = dependency('gtk4', required: true)
sdl_dep = dependency('sdl3', required: true)

# Source files
sources = [
  'src/main.c',
  'src/emulator.c',
  'src/emulator.h',
  'src/nes.c',
  'src/nes.h',
  'src/cpu.c',
  'src/cpu.h',
  'src/ppu.c',
  'src/ppu.h',
  'src/rom.c',
  'src/rom.h',
  'src/input.c',
  'src/input.h',
]

# Build executable
executable('nes83x',
  sources,
  dependencies: [gtk_dep, sdl_dep],
  install: true,
)
