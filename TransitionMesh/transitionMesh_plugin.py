# -*- coding: utf-8 -*-
# ============================================================================
#  Transition Mesh
#  created by IY,China University of Petroleum (East China)
#  禁止商用 / Non-commercial use only
#  禁止商用：本软件仅供学习与研究使用，未经作者书面许可不得用于任何商业用途。
#  Non-commercial use only. Unauthorized commercial use is prohibited.
# ============================================================================

from abaqusGui import *
from abaqusConstants import ALL
from kernelAccess import mdb, session
import hashlib
import inspect
import os
import sys


TM_BUILD = '2026.10.09-axial-bands.1'
thisPath = os.path.realpath(os.path.abspath(__file__))
thisDir = os.path.dirname(thisPath)
parentDir = os.path.dirname(thisDir)
if thisDir not in sys.path:
    sys.path.insert(0, thisDir)


def _install_canonical_loader(paths, hashes, role):
    """GUI 和内核共用的白名单加载器；仅管理本插件模块，不改 sys.path。"""
    import hashlib
    import importlib.abc
    import importlib.machinery
    import importlib.util
    import os
    import sys

    def canonical(path):
        return os.path.normcase(os.path.realpath(os.path.abspath(path)))

    def conflict(message):
        raise RuntimeError('TM-VERSION-CONFLICT: %s. Restart CAE with one '
                           'canonical TransitionMesh deployment.' % message)

    paths = dict((name, canonical(path)) for name, path in paths.items())

    class SourceLoader(importlib.machinery.SourceFileLoader):
        def get_code(self, fullname):
            # 直接编译源文件，避免同秒同大小更新误用旧 pyc。
            data = self.get_data(paths[fullname])
            if hashlib.sha256(data).hexdigest() != hashes[fullname]:
                conflict('source changed: ' + paths[fullname])
            return compile(data, paths[fullname], 'exec')

        def exec_module(self, module):
            module._tm_source_sha256 = hashes[module.__name__]
            super(SourceLoader, self).exec_module(module)

    class CanonicalFinder(importlib.abc.MetaPathFinder):
        def __init__(self):
            self.paths = paths
            self.hashes = hashes

        def find_spec(self, fullname, path=None, target=None):
            if fullname not in paths:
                return None
            return importlib.util.spec_from_file_location(
                fullname, paths[fullname], loader=SourceLoader(fullname, paths[fullname]))

        def verify(self):
            for name, filename in paths.items():
                try:
                    with open(filename, 'rb') as stream:
                        digest = hashlib.sha256(stream.read()).hexdigest()
                except OSError as exc:
                    conflict(str(exc))
                if digest != hashes[name]:
                    conflict('source changed: ' + filename)
            # 别名导入也检查，防止另一个根目录缓存了相同生产模块。
            for alias, module in list(sys.modules.items()):
                filename = getattr(module, '__file__', '') or ''
                stem = os.path.splitext(os.path.basename(filename))[0]
                name = alias.rsplit('.', 1)[-1]
                if name not in paths:
                    name = stem if stem in paths else None
                if name is None:
                    continue
                if not filename or canonical(filename) != paths[name]:
                    conflict('%s loaded from %s; expected %s'
                             % (alias, filename or '(unknown)', paths[name]))
                if getattr(module, '_tm_source_sha256', None) != hashes[name]:
                    # A module already imported from the canonical file -- e.g.
                    # CAE loads the loose root *_plugin.py library first -- carries no marker yet.
                    module._tm_source_sha256 = hashes[name]
                    print('TM-LOAD %s: adopted already-imported module %s'
                          % (role, alias))

    existing = getattr(sys, '_tm_canonical_loader', None)
    if existing is not None:
        if existing.paths != paths or existing.hashes != hashes:
            conflict('another build/root already owns this process')
        existing.verify()
        return existing
    finder = CanonicalFinder()
    finder.verify()
    finder.strict = True
    sys.meta_path.insert(0, finder)
    sys._tm_canonical_loader = finder
    if os.environ.get('TM_VERBOSE'):
        print('TM-LOAD %s: canonical root %s'
              % (role, os.path.dirname(paths['transition_mesh_plan'])))
    return finder


def _deployment_manifest():
    """只接受完整规范布局；扫描到备份注册器时拒绝注册且不影响其他插件。"""
    components = thisPath.replace('\\', '/').lower().split('/')
    if (os.path.basename(thisDir) != 'TransitionMesh'
            or os.path.basename(thisPath) != 'transitionMesh_plugin.py'
            or any('backup' in item or item == 'history' or item.startswith('problem2_')
                   for item in components)):
        raise RuntimeError('TM-VERSION-CONFLICT: refusing noncanonical/backup registrar: ' + thisPath)
    paths = dict((name, os.path.join(thisDir, name + '.py')) for name in (
        'transition_mesh_plan', 'transition_mesh_rules',
        'transition_mesh_runtime', 'transition_mesh_regions',
        'transition_mesh_corner_templates', 'transition_mesh_plugin',
        'transition_mesh_quad2', 'transition_mesh_quad2_cae',
        'transition_mesh_plugin_planar2d',
        'transition_mesh_plugin_cylinder_kernel',
        'transitionMesh_plugin', 'transitionMeshDB',
        'transition_mesh_gui_kernel'))
    hashes = {}
    for name, path in paths.items():
        if not os.path.isfile(path):
            raise RuntimeError('TM-VERSION-CONFLICT: incomplete deployment: ' + path)
        with open(path, 'rb') as stream:
            hashes[name] = hashlib.sha256(stream.read()).hexdigest()
    return paths, hashes


def _make_kernel_init(paths, hashes):
    """发送同一加载契约到独立内核进程，并核对所有延迟依赖。"""
    names = tuple(name for name in paths if name not in (
        'transitionMesh_plugin', 'transitionMeshDB'))
    return (inspect.getsource(_install_canonical_loader)
            + '\n_tm_kernel_loader = _install_canonical_loader(%r, %r, "KERNEL")\n'
            % (paths, hashes)
            + 'import importlib as _tm_importlib\n'
            + 'import os as _tm_os\n'
            + 'for _tm_name in %r:\n' % (names,)
            + '    _tm_module = _tm_importlib.import_module(_tm_name)\n'
            + '    if _tm_os.environ.get("TM_VERBOSE"):\n'
            + '        print("TM-KERNEL %s %s sha256=%s" % '
              '(_tm_name, _tm_module.__file__, _tm_module._tm_source_sha256))\n'
            + 'transition_mesh_gui_kernel = _tm_importlib.import_module("transition_mesh_gui_kernel")\n'
            + '_tm_kernel_loader.verify()\n')


class _TransitionTypeKeyword(AFXStringKeyword):
    """The DB dialog attaches ``tm_listener`` and uses it to switch the visible control groups when the Type"""

    tm_listener = None

    def handle(self, sender, sel, ptr):
        result = None
        try:
            result = AFXStringKeyword.handle(self, sender, sel, ptr)
        except Exception:
            pass
        try:
            fn = getattr(self, 'tm_listener', None)
            if fn is not None:
                fn()
        except Exception:
            pass
        return result


class TransitionMesh_plugin(AFXForm):

    def __init__(self, owner):
        AFXForm.__init__(self, owner)

        self.cmd = AFXGuiCommand(
            mode=self,
            method='run_transition_mesh_gui',
            objectName='transition_mesh_gui_kernel',
            registerQuery=False
        )

        pickedDefault = ''
        # Kept for script/backward compatibility;
        # the GUI no longer exposes Model/Part entries (target part is inferred from the viewport picks in
        self.modelNameKw = AFXStringKeyword(self.cmd, 'modelName', True, 'Model-1')
        self.partNameKw = AFXStringKeyword(self.cmd, 'partName', True, 'Part-1')

        # Transition type keyword; it tells the dialog which sections to show.
        try:
            self.transitionTypeKw = _TransitionTypeKeyword(
                self.cmd, 'transitionType', True, 'PLANAR')
        except Exception:
            self.transitionTypeKw = AFXStringKeyword(
                self.cmd, 'transitionType', True, 'PLANAR')
        # 2D element library code for PLANAR_ANNULAR / PLANAR_QUAD_FOUR.
        # Empty = keep the part's own element type (the GUI no longer assigns any element type;
        self.element2dKw = AFXStringKeyword(self.cmd, 'element2d', True,
                                            '')
        # PLANAR_QUAD_FOUR: Ny on the v-sides; 0 = use seeds (Nx) for both
        self.fineSeedsYKw = AFXIntKeyword(self.cmd, 'fineSeedsY', True, 0)
        # Mesh center + regular domain faces after transition (2D only)
        self.meshRegular2dKw = AFXBoolKeyword(
            self.cmd, 'meshRegular2d', AFXBoolKeyword.TRUE_FALSE, True, True
        )
        # Full 2D processing region (center + rings / center + outer frame).
        # Optional: without it only the transition band is touched.
        self.pickedDomainFacesKw = AFXObjectKeyword(
            self.cmd, 'pickedDomainFaces', TRUE, pickedDefault)

        # PLANAR selections
        self.pickedFaceKw = AFXObjectKeyword(self.cmd, 'pickedFace', TRUE, pickedDefault)
        self.pickedEdgeKw = AFXObjectKeyword(self.cmd, 'pickedEdge', TRUE, pickedDefault)
        # Classic PLANAR owns a separate optional coarse-edge channel.
        self.pickedPlanarCoarseEdgeKw = AFXObjectKeyword(
            self.cmd, 'pickedPlanarCoarseEdge', TRUE, pickedDefault)

        # Kept for script/backward compatibility; GUI no longer exposes them.
        self.facePointKw = AFXStringKeyword(self.cmd, 'facePoint', True, '')
        self.edgePointKw = AFXStringKeyword(self.cmd, 'edgePoint', True, '')
        self.modeKw = AFXStringKeyword(self.cmd, 'mode', True, 'PLANAR')
        self.orientationKw = AFXStringKeyword(self.cmd, 'orientation', True, 'HU')
        self.curvedSurfaceKw = AFXBoolKeyword(
            self.cmd, 'curvedSurface', AFXBoolKeyword.TRUE_FALSE, True, False
        )

        # Shared parameters (PLANAR and CYLINDER)
        self.depthKw = AFXFloatKeyword(self.cmd, 'depth', True, 5.0)
        # The public GUI calls this Stages. Keep ``layers`` as a script
        # compatibility keyword; the formal kernel contract consumes stages.
        self.stagesKw = AFXIntKeyword(self.cmd, 'stages', True, 1)
        self.layersKw = AFXIntKeyword(self.cmd, 'layers', True, 1)
        self.seedsKw = AFXIntKeyword(self.cmd, 'seeds', True, 4)
        self.ratioKw = AFXStringKeyword(self.cmd, 'ratio', True, '2:1')
        self.ratioSequenceKw = AFXStringKeyword(
            self.cmd, 'ratioSequence', True, '')
        self.stageWidthsKw = AFXStringKeyword(
            self.cmd, 'stageWidths', True, '')

        # Cylinder selections (ANNULAR_RADIAL / AXIAL_SURFACE)
        self.pickedRegionFacesKw = AFXObjectKeyword(
            self.cmd, 'pickedRegionFaces', TRUE, pickedDefault)
        self.pickedFineEdgesKw = AFXObjectKeyword(
            self.cmd, 'pickedFineEdges', TRUE, pickedDefault)
        self.pickedCoarseEdgesKw = AFXObjectKeyword(
            self.cmd, 'pickedCoarseEdges', TRUE, pickedDefault)
        self.pickedCellsKw = AFXObjectKeyword(
            self.cmd, 'pickedCells', TRUE, pickedDefault)
        self.pickedPathEdgesKw = AFXObjectKeyword(
            self.cmd, 'pickedPathEdges', TRUE, pickedDefault)
        self.actionKw = AFXStringKeyword(self.cmd, 'action', True,
                                         'PARTITION_SEED_MESH')
        # GUI-only integer selection; human-readable labels are not expressions.
        # Abaqus 2025 expects 'command', not 'cmd'; use positional arguments.
        self.actionChoiceKw = AFXIntKeyword(None, 'actionChoice', False, 1, False)
        # Check requests are matched by the GUI through these two values.
        # The kernel writes the result atomically to resultFile.
        self.requestIdKw = AFXStringKeyword(self.cmd, 'requestId', True, '')
        self.resultFileKw = AFXStringKeyword(self.cmd, 'resultFile', True, '')
        # Read-only Check: when True the kernel validates and returns
        # without partition/seed/mesh (spec 7). Always reset to False.
        self.checkOnlyKw = AFXBoolKeyword(
            self.cmd, 'checkOnly', AFXBoolKeyword.TRUE_FALSE, True, False
        )

        # Transition-edge-driven auto-adapt of the regular regions (adjacent bands + axial/radial layers).
        self.autoRegularKw = AFXBoolKeyword(
            self.cmd, 'autoRegular', AFXBoolKeyword.TRUE_FALSE, True, True
        )

        self.generateMeshKw = AFXBoolKeyword(
            self.cmd, 'generateMesh', AFXBoolKeyword.TRUE_FALSE, True, False
        )
        self.validateKw = AFXBoolKeyword(
            self.cmd, 'validate', AFXBoolKeyword.TRUE_FALSE, True, True
        )
        # Hybrid 模式：显式粗边种子数（0 = 由 ratio 自动推导）
        self.coarseSeedsKw = AFXIntKeyword(self.cmd, 'coarseSeeds', True, 0)
        # Hybrid 模式（GUI）：2:1 块数 b（0 = 未启用）。插件据此算
        # 3:1 块数 a=(N-2b)/3 与粗边数 M=(N+b)/3；b 必须为偶数。
        self.hybridBlocksKw = AFXIntKeyword(self.cmd, 'hybridBlocks', True, 0)

    # Canonical transition-type codes; legacy display strings that start with a canonical code map to it,
    # anything else is rejected (never falls through to PLANAR). Longer names first.
    _CANONICAL_TTYPES = (
        'PLANAR_ANNULAR', 'PLANAR_QUAD_FOUR',
        'PLANAR', 'ANNULAR_RADIAL', 'AXIAL_SURFACE',
    )

    @classmethod
    def _canonical_transition_type(cls, value):
        v = str(value).strip().upper()
        if v in cls._CANONICAL_TTYPES:
            return v
        for code in cls._CANONICAL_TTYPES:
            if v.startswith(code):
                return code
        return None

    def getFirstDialog(self):
        _gui_loader.verify()
        import transitionMeshDB
        _gui_loader.verify()
        try:
            import tempfile as _tf
            log_path = os.path.join(_tf.gettempdir(), 'TransitionMesh',
                                    'load.log')
        except Exception:
            log_path = 'load.log'
        print('Transition Mesh %s loaded (build check: %s; set TM_VERBOSE=1 '
              'for the module list)' % (TM_BUILD, log_path)) \
            if os.environ.get('TM_VERBOSE') else None
        previous = getattr(self, '_tm_dialog', None)
        self._tm_dialog = None
        if previous is not None:
            try:
                previous.discard()
            except Exception:
                pass
        self.tm_build = TM_BUILD
        self._tm_dialog = transitionMeshDB.TransitionMeshDB(self)
        self._reset_pick_keywords()
        for handler in getattr(self._tm_dialog, 'pick_handlers', []) or []:
            try:
                handler.refreshLabel()
            except Exception:
                pass
        return self._tm_dialog

    _PICK_KEYWORDS = (
        'pickedFaceKw', 'pickedEdgeKw', 'pickedPlanarCoarseEdgeKw',
        'pickedDomainFacesKw', 'pickedRegionFacesKw', 'pickedFineEdgesKw',
        'pickedCoarseEdgesKw', 'pickedCellsKw', 'pickedPathEdgesKw',
    )

    def _reset_pick_keywords(self):
        # AFXForm keeps keyword values across dialog instances. Geometry picks
        # are short-lived viewport state, so every fresh dialog starts with an
        # empty Selection panel while numeric/text parameters remain unchanged.
        for name in self._PICK_KEYWORDS:
            keyword = getattr(self, name, None)
            if keyword is None:
                continue
            # A deactivated keyword silently keeps its value; activate first so
            # the reset also clears picks retained from a previous Part.
            try:
                keyword.activate()
            except Exception:
                pass
            try:
                keyword.setValue('')
            except Exception:
                pass
        self._sync_pick_keyword_activity()

    def _active_pick_keyword_names(self):
        ttype = self._canonical_transition_type(self.transitionTypeKw.getValue())
        if ttype == 'PLANAR':
            return ('pickedFaceKw', 'pickedEdgeKw',
                    'pickedPlanarCoarseEdgeKw', 'pickedDomainFacesKw')
        if ttype in ('PLANAR_ANNULAR', 'PLANAR_QUAD_FOUR'):
            return ('pickedRegionFacesKw', 'pickedFineEdgesKw',
                    'pickedCoarseEdgesKw', 'pickedDomainFacesKw')
        if ttype in ('ANNULAR_RADIAL', 'AXIAL_SURFACE'):
            return ('pickedRegionFacesKw', 'pickedFineEdgesKw',
                    'pickedCoarseEdgesKw', 'pickedCellsKw', 'pickedPathEdgesKw')
        return ()

    def _sync_pick_keyword_activity(self):
        # Exclude hidden stale geometry BEFORE Python evaluates command arguments.
        # Preserve its value so switching back to a mode does not erase selections.
        active = self._active_pick_keyword_names()
        for name in self._PICK_KEYWORDS:
            keyword = getattr(self, name)
            if name in active:
                keyword.activate()
            else:
                keyword.deactivate()

    def getCommandString(self):
        # 每次执行再核对 GUI 与内核，防止激活后插入旧模块或替换部署文件。
        _gui_loader.verify()
        self._sync_pick_keyword_activity()
        command = AFXForm.getCommandString(self)
        if not command or not command.strip():
            raise RuntimeError('No Transition Mesh command was constructed.')
        # Abaqus sometimes emits the object expression for pickedPathEdges but
        # omits the keyword argument from the generated command. Preserve the
        # user's GUI pick explicitly for cylinder modes.
        ttype = self._canonical_transition_type(self.transitionTypeKw.getValue())
        if (ttype in ('ANNULAR_RADIAL', 'AXIAL_SURFACE')
                and 'pickedPathEdges=' not in command
                and 'pickedPickedPathEdges' in command
                and ', action=' in command):
            command = command.replace(
                ', action=',
                ', pickedPathEdges=pickedPickedPathEdges, action=', 1)
        if getattr(_gui_loader, 'strict', False):
            command = '_tm_kernel_loader.verify()\n' + command
        # The wrapper catches errors in loader verification and pick expressions,
        # which occur BEFORE run_transition_mesh_gui can publish a result.
        guarded = ('transition_mesh_gui_kernel.dispatch_transition_mesh_command('
                   'command=%r, request_id=%r, result_file=%r, '
                   'feedback=%r, namespace=globals())' % (
                       command, self.requestIdKw.getValue(),
                       self.resultFileKw.getValue(),
                       bool(self.checkOnlyKw.getValue()) or
                       str(self.actionKw.getValue()).upper() in ('CHECK_ONLY', 'PRE_MESH')))
        self._tm_dispatch_count = getattr(self, '_tm_dispatch_count', 0) + 1
        return guarded

    def handleException(self, exc):
        # created by IY,China University of Petroleum (East China)
        dialog = getattr(self, '_tm_dialog', None)
        if dialog is not None and not getattr(dialog, '_tm_closed', False):
            try:
                dialog.onKernelException(exc)
            except Exception:
                pass
        AFXForm.handleException(self, exc)

    def doCustomTasks(self):
        # Documented callback after successful processing, not just command creation.
        dialog = getattr(self, '_tm_dialog', None)
        if dialog is not None and not getattr(dialog, '_tm_closed', False):
            dialog._poll_check_result()

    def deactivate(self):
        dialog = getattr(self, '_tm_dialog', None)
        self._tm_dialog = None
        if dialog is not None:
            try:
                dialog.shutdown()
            except Exception:
                try:
                    dialog.discard()
                except Exception:
                    pass
        AFXForm.deactivate(self)

    def doCustomChecks(self):
        if (getattr(self, '_tm_check_running', False)
                and not getattr(self, '_tm_check_dispatching', False)):
            dialog = getattr(self, '_tm_dialog', None)
            title = ('Pre-mesh' if dialog is not None
                     and dialog._check_kind == 'PRE_MESH' else 'Check')
            showAFXErrorDialog(
                getAFXApp().getAFXMainWindow(),
                '%s is still running. Wait for its result before applying.' % title
            )
            return False
        # Commit the native combo choice before validation/command creation.
        # Pre-mesh temporarily owns actionKw during its own dispatch.
        dialog = getattr(self, '_tm_dialog', None)
        if dialog is not None and not getattr(self, '_tm_check_dispatching', False):
            dialog._sync_action_keyword()
            dialog._invalidate_check_if_stale()
        self._sync_pick_keyword_activity()
        # A FAIL verdict from the latest Check blocks Apply until the inputs change (the dialog clears the flag;
        # the kernel still audits itself).
        if (getattr(self, '_tm_block_apply', False)
                and not getattr(self, '_tm_check_dispatching', False)):
            showAFXErrorDialog(
                getAFXApp().getAFXMainWindow(),
                'The last Check failed for these inputs. Fix the reported '
                'problem, then run Check again before applying.'
            )
            return False
        # 在检查前同步当前视口的 Model/Part 到关键字（作为推导回退）
        self._sync_viewport_target()
        _gui_loader.verify()
        ttype = self._canonical_transition_type(
            self.transitionTypeKw.getValue())
        if ttype is None:
            showAFXErrorDialog(
                getAFXApp().getAFXMainWindow(),
                'Unknown transition type. Use PLANAR, PLANAR_ANNULAR, '
                'PLANAR_QUAD_FOUR, ANNULAR_RADIAL or AXIAL_SURFACE.'
            )
            return False
        try:
            stages = int(self.stagesKw.getValue())
        except Exception:
            showAFXErrorDialog(
                getAFXApp().getAFXMainWindow(),
                'Stages must be a positive integer.'
            )
            return False
        if stages < 1:
            showAFXErrorDialog(
                getAFXApp().getAFXMainWindow(),
                'Stages must be a positive integer.'
            )
            return False
        if ttype in ('ANNULAR_RADIAL', 'AXIAL_SURFACE'):
            missing = []
            for kw, label in (
                    (self.pickedRegionFacesKw, 'Region Faces'),
                    (self.pickedFineEdgesKw, 'Fine Edge Loop'),
                    (self.pickedCoarseEdgesKw, 'Coarse Edge Loop'),
                    (self.pickedCellsKw, 'Target Cells')):
                v = kw.getValue()
                if not v or v[0] == '<':
                    missing.append(label)
            if missing:
                showAFXErrorDialog(
                    getAFXApp().getAFXMainWindow(),
                    'Cylinder transition (%s) needs: %s'
                    % (ttype, ', '.join(missing))
                )
                return False
            if self.seedsKw.getValue() <= 0:
                showAFXErrorDialog(
                    getAFXApp().getAFXMainWindow(),
                    'Seeds Number must be positive.'
                )
                return False
            return True

        if ttype in ('PLANAR_ANNULAR', 'PLANAR_QUAD_FOUR'):
            # The fine loop is always required.
            # The coarse loop is required for PLANAR_ANNULAR only:
            missing = []
            for kw, label in (
                    (self.pickedRegionFacesKw, 'Transition Face'),
                    (self.pickedFineEdgesKw, 'Fine Edge Loop'),
                    (self.pickedCoarseEdgesKw, 'Coarse Edge Loop')):
                v = kw.getValue()
                if not v or v[0] == '<':
                    missing.append(label)
            v = self.pickedFineEdgesKw.getValue()
            v2 = self.pickedCoarseEdgesKw.getValue()
            fine_missing = not v or v[0] == '<'
            coarse_missing = not v2 or v2[0] == '<'
            if fine_missing or (ttype == 'PLANAR_ANNULAR' and coarse_missing):
                showAFXErrorDialog(
                    getAFXApp().getAFXMainWindow(),
                    '%s needs the Fine Edge Loop%s.'
                    % (ttype, ' and the Coarse Edge Loop'
                       if ttype == 'PLANAR_ANNULAR' else '')
                )
                return False
            if self.seedsKw.getValue() <= 0:
                showAFXErrorDialog(
                    getAFXApp().getAFXMainWindow(),
                    'Seeds Number must be positive.'
                )
                return False
            # 多层整除条件及角部计数由内核规划器统一校验，保留 FAIL 的建议值。
            if ttype == 'PLANAR_QUAD_FOUR' and self.fineSeedsYKw.getValue() < 0:
                showAFXErrorDialog(getAFXApp().getAFXMainWindow(),
                                   'Ny must be positive, or 0 for Same as Nx.')
                return False
            return True

        # PLANAR checks
        picked_face = self.pickedFaceKw.getValue()
        has_face_pick = picked_face and picked_face[0] != '<'
        picked_edge = self.pickedEdgeKw.getValue()
        has_edge_pick = picked_edge and picked_edge[0] != '<'
        if not has_face_pick:
            showAFXErrorDialog(
                getAFXApp().getAFXMainWindow(),
                'Pick a Partition Face from the viewport.'
            )
            return False
        if not has_edge_pick:
            showAFXErrorDialog(
                getAFXApp().getAFXMainWindow(),
                'Pick a Finer Edge from the viewport for PLANAR transition.'
            )
            return False
        if self.seedsKw.getValue() <= 0:
            showAFXErrorDialog(
                getAFXApp().getAFXMainWindow(),
                'Seeds Number must be positive.'
            )
            return False
        if self.depthKw.getValue() <= 0.0:
            showAFXErrorDialog(
                getAFXApp().getAFXMainWindow(),
                'Depth must be positive.'
            )
            return False
        return True

    def _sync_viewport_target(self):
        """这确保即使用户从非 Model-1 的 Part 上拾取，内核也能获得正确的目标归属。不会覆盖用户显式设置的值。"""
        try:
            vp_name = session.currentViewportName
            vp = session.viewports[vp_name]
            obj = vp.displayedObject
            if obj is None:
                return
            # displayedObject 可能是 Part 或 Assembly
            part_name = getattr(obj, 'name', None)
            model_name = getattr(obj, 'modelName', None)
            if part_name and model_name:
                self.modelNameKw.setValue(model_name)
                self.partNameKw.setValue(part_name)
        except Exception:
            pass

    def okToCancel(self):
        return False


class _PlainLoader(object):
    """Used only when the strict canonical setup cannot be installed (for example an unexpected host layout)."""

    strict = False
    menu_registered = False

    def __init__(self, paths):
        self.paths = paths
        self.hashes = {}

    def verify(self):
        return True


def _tm_log_line(text):
    # Append to <temp>\TransitionMesh\load.log; only echo to the message
    # area when TM_VERBOSE is set, so opening the plugin stays quiet.
    try:
        if os.environ.get('TM_VERBOSE'):
            print(text)
    except Exception:
        pass
    try:
        import tempfile
        folder = os.path.join(tempfile.gettempdir(), 'TransitionMesh')
        if not os.path.isdir(folder):
            os.makedirs(folder)
        with open(os.path.join(folder, 'load.log'), 'a') as stream:
            stream.write(str(text) + '\n')
    except Exception:
        pass


def _register_menu(loader, kernel_init):
    """Register the menu once; shared by the strict and fallback paths."""
    if getattr(loader, 'menu_registered', False):
        return
    toolset = getAFXApp().getAFXMainWindow().getPluginToolset()
    toolset.registerGuiMenuButton(
        buttonText='Transition Mesh',
        object=TransitionMesh_plugin(toolset),
        messageId=AFXMode.ID_ACTIVATE,
        icon=None,
        kernelInitString=kernel_init,
        applicableModules=('Part', 'Mesh'),
        version=TM_BUILD,
        author='IY, China University of Petroleum (East China)',
        description='Five transition modes; shared Stages; read-only Check.',
        helpUrl=''
    )
    loader.menu_registered = True


# 注册失败仅阻止本插件；不删除部署、不改其他插件的导入路径。
_tm_source_sha256 = 'n/a'
register_warning = None
try:
    _tm_paths, _tm_hashes = _deployment_manifest()
    _tm_source_sha256 = _tm_hashes['transitionMesh_plugin']
    _gui_loader = _install_canonical_loader(_tm_paths, _tm_hashes, 'GUI')
    _tm_kernel_init = _make_kernel_init(_tm_paths, _tm_hashes)
except Exception as exc:
    # Strict setup unavailable: keep the plugin usable with the legacy
    # registration and surface the reason in the console and load.log.
    register_warning = 'strict loader unavailable: %s' % exc
    _tm_paths, _tm_hashes = {}, {}
    _gui_loader = _PlainLoader(_tm_paths)
    _tm_kernel_init = ('import sys; sys.path.insert(0, %r); '
                       'import transition_mesh_gui_kernel' % thisDir)
    if thisDir not in sys.path:
        sys.path.insert(0, thisDir)
    _tm_log_line('TM-LOAD WARNING: %s' % register_warning)
try:
    _register_menu(_gui_loader, _tm_kernel_init)
    _tm_log_line('TM-GUI %s %s sha256=%s' % (TM_BUILD, thisPath, _tm_source_sha256))
except Exception as exc:
    _tm_log_line('TM-REGISTER FAILED: %s' % exc)
