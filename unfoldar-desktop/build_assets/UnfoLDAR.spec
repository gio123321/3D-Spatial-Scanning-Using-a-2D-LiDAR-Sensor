# -*- mode: python ; coding: utf-8 -*-
from PyInstaller.utils.hooks import collect_submodules
from PyInstaller.utils.hooks import collect_all
from pathlib import Path
import os

root = Path(SPECPATH).parent
os.chdir(root)

test_modules = ('pytest', '_pytest', 'pytest_cov', 'pytestqt', 'coverage',
                'pyvista._pytest_plugin', 'IPython.testing.plugin',
                'IPython.testing.tools', 'IPython.testing.ipunittest')

def is_test_module(name):
    return any(name == prefix or name.startswith(prefix + '.') for prefix in test_modules)

datas = [('src/unfoldar/resources', 'unfoldar/resources'),
         ('profiles/ov2640_unfoldar_final.json', 'profiles')]
datas = [(str(root / source), target) for source, target in datas]
binaries = []
hiddenimports = ['vtkmodules.util.numpy_support', 'vtkmodules.qt', 'PySide6.QtSvg', 'PySide6.QtSvgWidgets']
hiddenimports += collect_submodules('vtkmodules')
for package in ('open3d', 'pyvista', 'pyvistaqt', 'xatlas'):
    package_data, package_binaries, package_imports = collect_all(package)
    datas.extend(package_data)
    binaries.extend(package_binaries)
    hiddenimports.extend(package_imports)

hiddenimports = [name for name in hiddenimports if not is_test_module(name)]


a = Analysis(
    [str(root / 'build_assets/launch.py')],
    pathex=[str(root / 'src')],
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=list(test_modules),
    noarchive=False,
    optimize=0,
)
fixture_modules = ('pyvista.conftest', 'vtkmodules.test', 'vtkmodules.web.testing')
a.pure = [entry for entry in a.pure
          if not any(entry[0] == name or entry[0].startswith(name + '.') for name in fixture_modules)]
assert not any(is_test_module(entry[0]) for entry in a.pure), 'Test module entered frozen runtime'
pyz = PYZ(a.pure)

exe = EXE(
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name='UnfoLDAR Desktop',
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=False,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
    icon=[str(root / 'build_assets/app_icon.ico')],
)
coll = COLLECT(
    exe,
    a.binaries,
    a.datas,
    strip=False,
    upx=False,
    upx_exclude=[],
    name='UnfoLDAR Desktop',
)
