# -*- coding: utf-8 -*-
# ============================================================================
#  Transition Mesh
#  created by IY,China University of Petroleum (East China)
#  禁止商用 / Non-commercial use only
#  禁止商用：本软件仅供学习与研究使用，未经作者书面许可不得用于任何商业用途。
#  Non-commercial use only. Unauthorized commercial use is prohibited.
# ============================================================================

# Hand-edited dialog code: reopening it in RSG Dialog Builder would drop the
# dynamic show/hide of the control groups of the current transition type.

from abaqusConstants import *
from abaqusGui import *
from kernelAccess import mdb, session
import json
import os
import tempfile
import time
import uuid

import abaqusGui as _ag

# Official AFXPickStep class flags (int-valued GUI enums). Kernel SymbolicConstants (abaqusConstants.
# ALL / FACES / EDGES / CELLS / TUPLE ...
_PICK = _ag.AFXPickStep


thisPath = os.path.abspath(__file__)
thisDir = os.path.dirname(thisPath)
TM_AUTHOR_CREDIT = '(created by IY,China University of Petroleum (East China))'
TM_SUBTITLE = TM_AUTHOR_CREDIT
TM_WINDOW_TITLE = 'Transition Mesh V1.0 ' + TM_AUTHOR_CREDIT


def _canonical_type(value):
    """Same canonicalization as the kernel dispatcher."""
    v = str(value or '').strip().upper()
    for code in ('PLANAR_ANNULAR', 'PLANAR_QUAD_FOUR',
                 'PLANAR', 'ANNULAR_RADIAL', 'AXIAL_SURFACE'):
        if v == code or v.startswith(code):
            return code
    return None


class TransitionMeshDB(AFXDataDialog):

    # Custom selectors.
    ID_CHECK = AFXMode.ID_LAST + 101
    ID_SUGGEST = AFXMode.ID_LAST + 102
    ID_PREMESH = AFXMode.ID_LAST + 104
    ID_HYBRID = AFXMode.ID_LAST + 105
    ID_FIT = AFXMode.ID_LAST + 106


    # UI labels deliberately differ from the command values.
    # The old PARTITION_ONLY value remains script-compatible but is not a primary UI action in the two-action
    _ACTION_ITEMS = (
        ('Partition + seed only (no mesh)', 'PARTITION_AND_SEED'),
        ('Partition + seed + mesh', 'PARTITION_SEED_MESH'),
    )

    # Check lifecycle: how long the fixed bar waits for the matching kernel
    # JSON before declaring a timeout (seconds; override TM_CHECK_TIMEOUT).
    _CHECK_TIMEOUT_ENV = 'TM_CHECK_TIMEOUT'
    _CHECK_TIMEOUT_DEFAULT = 120.0

    def __init__(self, form):
        self.form = form
        self._tm_closed = False
        AFXDataDialog.__init__(
            self, form, TM_WINDOW_TITLE,
            self.OK | self.APPLY | self.CANCEL,
            DIALOG_ACTIONS_SEPARATOR | DECOR_RESIZE
        )
        # Check button: temporarily run action=CHECK_ONLY, restore after.
        try:
            FXMAPFUNC(self, SEL_COMMAND, TransitionMeshDB.ID_CHECK,
                      TransitionMeshDB.onCheck)
            FXMAPFUNC(self, SEL_COMMAND, TransitionMeshDB.ID_PREMESH,
                      TransitionMeshDB.onPreMesh)
            FXMAPFUNC(self, SEL_COMMAND, TransitionMeshDB.ID_SUGGEST,
                      TransitionMeshDB.onSuggestValues)
            FXMAPFUNC(self, SEL_COMMAND, TransitionMeshDB.ID_HYBRID,
                      TransitionMeshDB.onHybridToggled)
            FXMAPFUNC(self, SEL_TIMEOUT, TransitionMeshDB.ID_FIT,
                      TransitionMeshDB.onCmdFit)
        except Exception:
            pass

        # OK and Apply both dispatch the same command (run_transition_mesh_gui with the current Action);
        # only OK closes the dialog. Keep the stock labels so no button promises a partition-only run.
        self.pick_handlers = []
        self._check_request_id = None
        self._check_result_file = None
        self._check_snapshot = None
        self._check_result = None
        self._check_dispatching = False
        self._last_check_read_error = None
        self._check_started_at = None
        self._check_kind = 'CHECK'
        self._suggest_after = False
        self._hybrid_on = False
        try:
            self.form._tm_check_running = False
            self.form._tm_check_dispatching = False
        except Exception:
            pass

        # Scrollable body + pinned bottom bar,
        # following the pattern proven in the sibling rveVoidGen plugin (Abaqus 2025 AFX/FOX):
        self._root = FXPacker(
            p=self, opts=LAYOUT_FILL_X | LAYOUT_FILL_Y,
            pl=0, pr=0, pt=0, pb=0)
        # Fixed bottom bar (Check / status / Action) packed first at the bottom.
        self._bottomBar = FXVerticalFrame(
            p=self._root, opts=LAYOUT_FILL_X | LAYOUT_SIDE_BOTTOM,
            x=0, y=0, w=0, h=0, pl=4, pr=4, pt=2, pb=4)
        try:
            avail = int(getAFXApp().getAFXMainWindow().getHeight())
        except Exception:
            avail = 0
        try:
            # FXScrollWindow scrolls via the VSCROLLER/HSCROLLER options;
            scrollOpts = (LAYOUT_FILL_X | LAYOUT_FILL_Y |
                          VSCROLLER_ALWAYS | HSCROLLER_NEVER)
            self.mainScroll = FXScrollWindow(self._root, scrollOpts,
                                             0, 0, 0, 0)
            main_parent = self.mainScroll
            # Initial viewport hint; onCmdFit refines it to the real content.
            if avail > 300:
                try:
                    self.mainScroll.setHeight(int(avail * 0.55))
                except Exception:
                    pass
        except Exception:
            self.mainScroll = None
            main_parent = self._root
        # Content frame fills width only, so its height equals the full content
        # and the scroll window reports a real getContentHeight() to scroll on.
        main = FXVerticalFrame(
            p=main_parent, opts=LAYOUT_FILL_X, x=0, y=0, w=0, h=0,
            pl=4, pr=4, pt=4, pb=4
        )
        self._main = main

        FXLabel(p=main, text=TM_SUBTITLE,
                ic=None, opts=JUSTIFY_LEFT)

        # ---- target part --------------------------------------------------- No Model/Part name entry:
        # the target part is inferred from the viewport picks (transition_mesh_gui_kernel._infer_model_part).
        FXLabel(p=main,
                text='Target part is determined by the viewport picks below.',
                ic=None, opts=JUSTIFY_LEFT)

        # ---- transition type (shared) ---------------------------------------
        typeBox = FXGroupBox(p=main, text='Transition type',
                             opts=FRAME_GROOVE | LAYOUT_FILL_X)
        typeAligner = AFXVerticalAligner(
            p=typeBox, opts=LAYOUT_FILL_X, x=0, y=0, w=0, h=0,
            pl=0, pr=0, pt=0, pb=0
        )
        typeCombo = AFXComboBox(p=typeAligner, ncols=0, nvis=1,
                                text='Type:', tgt=form.transitionTypeKw, sel=0)
        typeCombo.setMaxVisible(8)
        # Combo values MUST be the canonical codes the kernel dispatches on;
        # explanations live in the hint label below, never in the item text.
        for item in ('PLANAR', 'PLANAR_ANNULAR', 'PLANAR_QUAD_FOUR',
                     'ANNULAR_RADIAL', 'AXIAL_SURFACE'):
            typeCombo.appendItem(text=item)
        FXLabel(p=typeAligner,
                text='Sections below follow the selected Type.',
                ic=None, opts=JUSTIFY_LEFT)

        # ---- selections: PLANAR (classic straight-edge) ----------------------
        self.selPlanar = FXGroupBox(
            p=main, text='Selections - PLANAR (classic)',
            opts=FRAME_GROOVE | LAYOUT_FILL_X)
        self._add_pick_button(
            parent=self.selPlanar,
            labelText='Partition Face',
            keyword=form.pickedFaceKw,
            prompt='Pick one planar face for the transition mesh',
            entitiesToPick=_PICK.FACES
        )
        self._add_pick_button(
            parent=self.selPlanar,
            labelText='Finer Edge',
            keyword=form.pickedEdgeKw,
            prompt='Pick the finer edge for the transition mesh',
            entitiesToPick=_PICK.EDGES
        )
        # Optional coarse boundary. When picked it owns the transition width:
        # Depth becomes the real distance finer edge -> this edge, and the typed Depth value is ignored.
        self._add_pick_button(
            parent=self.selPlanar,
            labelText='Coarse Edge (optional)',
            keyword=form.pickedPlanarCoarseEdgeKw,
            prompt='Optionally pick the coarse boundary edge on the same face',
            entitiesToPick=_PICK.EDGES
        )
        self.planarCoarsePick = self.pick_handlers[-1].label
        # Full domain is useful for pure-2D PLANAR regular-region meshing (spec 6.2).
        # It is shown for classic PLANAR and simply ignored by the kernel on a 3D part,
        self._add_pick_button(
            parent=self.selPlanar,
            labelText='Full domain (2D optional)',
            keyword=form.pickedDomainFacesKw,
            prompt='Pick full 2D domain faces for regular-region meshing',
            entitiesToPick=_PICK.FACES, numberToPick=_PICK.MANY
        )
        self.planarDomainPick = self.pick_handlers[-1].label

        # ---- selections: planar face modes (annular / quad four-side) --------
        self.selP2D = FXGroupBox(
            p=main, text='Selections - PLANAR face modes (pick, then Done)',
            opts=FRAME_GROOVE | LAYOUT_FILL_X)
        self._add_pick_button(
            parent=self.selP2D,
            labelText='1) Transition region',
            keyword=form.pickedRegionFacesKw,
            prompt='Pick transition band face(s)',
            entitiesToPick=_PICK.FACES, numberToPick=_PICK.MANY
        )
        self._add_pick_button(
            parent=self.selP2D,
            labelText='2) Fine loop',
            keyword=form.pickedFineEdgesKw,
            prompt='Pick fine edge loop (inner ring or inner square)',
            entitiesToPick=_PICK.EDGES, numberToPick=_PICK.MANY
        )
        self._add_pick_button(
            parent=self.selP2D,
            labelText='3) Coarse loop',
            keyword=form.pickedCoarseEdgesKw,
            prompt='Pick coarse edge loop (outer ring or outer square)',
            entitiesToPick=_PICK.EDGES, numberToPick=_PICK.MANY
        )
        self._add_pick_button(
            parent=self.selP2D,
            labelText='4) Full domain',
            keyword=form.pickedDomainFacesKw,
            prompt='Pick full 2D domain (center + rings / center + outer frame)',
            entitiesToPick=_PICK.FACES, numberToPick=_PICK.MANY
        )

        # ---- selections: CYLINDER -------------------------------------------
        self.selCyl = FXGroupBox(
            p=main, text='Selections - CYLINDER 3D (pick in this order)',
            opts=FRAME_GROOVE | LAYOUT_FILL_X)
        self._add_pick_button(
            parent=self.selCyl,
            labelText='1) Region Faces - transition band (multi)',
            keyword=form.pickedRegionFacesKw,
            prompt='Pick the transition band faces',
            entitiesToPick=_PICK.FACES, numberToPick=_PICK.MANY
        )
        self._add_pick_button(
            parent=self.selCyl,
            labelText='2) Fine Edge Loop (multi)',
            keyword=form.pickedFineEdgesKw,
            prompt='Pick the FINE circular edge loop (finer mesh side)',
            entitiesToPick=_PICK.EDGES, numberToPick=_PICK.MANY
        )
        self._add_pick_button(
            parent=self.selCyl,
            labelText='3) Coarse Edge Loop (multi)',
            keyword=form.pickedCoarseEdgesKw,
            prompt='Pick the COARSE circular edge loop (coarser mesh side)',
            entitiesToPick=_PICK.EDGES, numberToPick=_PICK.MANY
        )
        self._add_pick_button(
            parent=self.selCyl,
            labelText='4) Target Cells to mesh (multi)',
            keyword=form.pickedCellsKw,
            prompt='Pick the transition band cells to mesh',
            entitiesToPick=_PICK.CELLS, numberToPick=_PICK.MANY
        )
        self._add_pick_button(
            parent=self.selCyl,
            labelText='5) Sweep Path Edges (optional direction refs)',
            keyword=form.pickedPathEdgesKw,
            prompt='Pick straight sweep-direction reference edges',
            entitiesToPick=_PICK.EDGES, numberToPick=_PICK.MANY
        )

        # ---- shared transition parameters ------------------------------------ Layers / ratio / seeds apply
        # to EVERY Type.
        self.paramShared = FXGroupBox(
            p=main, text='Transition (all types)',
            opts=FRAME_GROOVE | LAYOUT_FILL_X)
        sharedAligner = AFXVerticalAligner(
            p=self.paramShared, opts=LAYOUT_FILL_X, x=0, y=0, w=0, h=0,
            pl=0, pr=0, pt=0, pb=0
        )
        self.seedsLabel = FXLabel(
            p=sharedAligner,
            text='Fine count N (fine edge / fine loop):',
            ic=None, opts=JUSTIFY_LEFT)
        AFXTextField(p=sharedAligner, ncols=10, labelText='Seeds:',
                     tgt=form.seedsKw, sel=0)
        # PLANAR_QUAD_FOUR only: Ny sits right next to Nx.  Empty/0 = use Nx
        # (the kernel already treats fineSeedsY=0 as "same as seeds").
        self.nyField = AFXTextField(
            p=sharedAligner, ncols=10, labelText='Ny (0 = same as Nx):',
            tgt=form.fineSeedsYKw, sel=0)
        self.layersField = AFXTextField(
            p=sharedAligner, ncols=10, labelText='Layers:',
            tgt=form.stagesKw, sel=0)
        ratioCombo = AFXComboBox(p=sharedAligner, ncols=0, nvis=1,
                                 text='Ratio:', tgt=form.ratioKw, sel=0)
        ratioCombo.setMaxVisible(4)
        for item in ('2:1', '3:1'):
            ratioCombo.appendItem(text=item)
        self.ratioCombo = ratioCombo
        # Live capability verdict, computed by the same rules module the kernel Check uses,
        # so the dialog can never promise what Check refuses. Read-only, no keyword binding.
        self.capabilityLabel = FXLabel(
            p=sharedAligner, text='Capability: --',
            ic=None, opts=JUSTIFY_LEFT)
        # Per-type summary (fine size -> target coarse size, chain, width).
        # Written by _update_computed_info;
        self.sharedInfoLabel = FXLabel(
            p=sharedAligner, text='Run Check to validate the selection.',
            ic=None, opts=JUSTIFY_LEFT)
        # Ratio sequence / stage widths remain script-compatible keywords but are no longer shown:
        # with one repeated ratio they are redundant (empty sequence == repeat the Ratio for every layer).

        # ---- parameters: PLANAR classic --------------------------------------
        self.paramPlanar = FXGroupBox(
            p=main, text='Parameters - PLANAR (classic)',
            opts=FRAME_GROOVE | LAYOUT_FILL_X)
        planarParamAligner = AFXVerticalAligner(
            p=self.paramPlanar, opts=LAYOUT_FILL_X, x=0, y=0, w=0, h=0,
            pl=0, pr=0, pt=0, pb=0
        )
        orientCombo = AFXComboBox(
            p=planarParamAligner, ncols=0, nvis=1, text='Orientation:',
            tgt=form.orientationKw, sel=0
        )
        orientCombo.setMaxVisible(6)
        for item in ('HU', 'HD', 'VR', 'VL'):
            orientCombo.appendItem(text=item)
        AFXTextField(p=planarParamAligner, ncols=8, labelText='Depth:',
                     tgt=form.depthKw, sel=0)
        # Hybrid: b is the 2:1 block count, a=(N-2b)/3 and M=(N+b)/3 are derived.
        # Regular-region meshing follows the Full Domain pick.
        self.hybridBtn = FXCheckButton(
            p=planarParamAligner, text='Hybrid (mixed 3:1 + 2:1)',
            tgt=self, sel=TransitionMeshDB.ID_HYBRID)
        self.hybridBlocksField = AFXTextField(
            p=planarParamAligner, ncols=10, labelText='2:1 blocks b:',
            tgt=form.hybridBlocksKw, sel=0)

        # ---- parameters: PLANAR 2D ------------------------------------------- The pure-2D modes
        # (PLANAR_ANNULAR / PLANAR_QUAD_FOUR) need no extra parameter box: Nx/Ny,

        # ---- parameters: CYLINDER --------------------------------------------
        self.paramCyl = FXGroupBox(
            p=main, text='Parameters - CYLINDER 3D',
            opts=FRAME_GROOVE | LAYOUT_FILL_X)
        cylAligner = AFXVerticalAligner(
            p=self.paramCyl, opts=LAYOUT_FILL_X, x=0, y=0, w=0, h=0,
            pl=0, pr=0, pt=0, pb=0
        )
        FXCheckButton(p=cylAligner,
                      text='Auto-adapt regular regions (seed + mesh)',
                      tgt=form.autoRegularKw, sel=0)
        FXLabel(p=cylAligner,
                text='Auto-adapt derives adjacent-band and layer counts from '
                     'the fine/coarse edge.',
                ic=None, opts=JUSTIFY_LEFT)
        FXLabel(p=cylAligner,
                text='AXIAL_SURFACE / ANNULAR_RADIAL support 2:1 and 3:1.',
                ic=None, opts=JUSTIFY_LEFT)
        FXLabel(p=cylAligner,
                text='Fine/coarse loops and template edges are seeded '
                     'automatically.',
                ic=None, opts=JUSTIFY_LEFT)

        # (Shared Seeds/Stages/ratio group moved above the per-Type groups
        # so the layer input is always visible; see 'Transition parameters'.)

        # ---- fixed bottom bar (outside scroll): Check / status / Action ----
        bot = self._bottomBar
        checkRow = FXHorizontalFrame(
            p=bot, opts=LAYOUT_FILL_X, x=0, y=0, w=0, h=0,
            pl=0, pr=0, pt=0, pb=0)
        self.checkBtn = FXButton(
            p=checkRow, text='Check',
            ic=None, tgt=self, sel=TransitionMeshDB.ID_CHECK,
            opts=BUTTON_NORMAL | LAYOUT_CENTER_Y, x=0, y=0, w=0, h=0,
            pl=6, pr=6, pt=2, pb=2)
        self.suggestBtn = FXButton(
            p=checkRow, text='Suggest values',
            ic=None, tgt=self, sel=TransitionMeshDB.ID_SUGGEST,
            opts=BUTTON_NORMAL | LAYOUT_CENTER_Y, x=0, y=0, w=0, h=0,
            pl=6, pr=6, pt=2, pb=2)
        self.preMeshBtn = FXButton(
            p=checkRow, text='Pre-mesh',
            ic=None, tgt=self, sel=TransitionMeshDB.ID_PREMESH,
            opts=BUTTON_NORMAL | LAYOUT_CENTER_Y, x=0, y=0, w=0, h=0,
            pl=6, pr=6, pt=2, pb=2)
        self.actionCombo = AFXComboBox(
            p=checkRow, ncols=0, nvis=2, text='Action:',
            tgt=form.actionChoiceKw, sel=0)
        self.actionCombo.setMaxVisible(len(self._ACTION_ITEMS))
        for idx, item in enumerate(self._ACTION_ITEMS):
            self.actionCombo.appendItem(text=item[0], sel=idx)
        self._sync_action_combo()
        self.checkLabel = FXLabel(
            p=bot,
            text='Check: not run yet.',
            ic=None, opts=JUSTIFY_LEFT)
        self.checkDetailLabel = FXLabel(
            p=bot,
            text='Run Check to validate the selected geometry and counts.',
            ic=None, opts=JUSTIFY_LEFT)
        self._check_fingerprint = None

        # ---- runtime section switching ---------------------------------------
        self._install_type_listener()
        self._apply_type_visibility()
        # AFX enlarges/restores the window after construction; refit deferred so
        # it matches the real content (two shots for late framework resizes).
        try:
            getAFXApp().addTimeout(100, self, self.ID_FIT)
            getAFXApp().addTimeout(600, self, self.ID_FIT)
        except Exception:
            pass

    # Type-driven section visibility (spec section 8)

    def _install_type_listener(self):
        """Attach the visibility refresh to the transitionType keyword."""
        kw = self.form.transitionTypeKw
        try:
            kw.tm_listener = self._on_type_changed
        except Exception:
            pass

    def _on_type_changed(self):
        self._apply_type_visibility()

    @staticmethod
    def _safe_value(value):
        try:
            return str(value)
        except Exception:
            return '<unavailable>'

    def _form_snapshot(self):
        """Return the GUI-side input snapshot used to reject stale results."""
        form = self.form
        names = (
            'modelNameKw', 'partNameKw', 'transitionTypeKw', 'modeKw',
            'orientationKw', 'depthKw', 'layersKw', 'stagesKw', 'seedsKw',
            'fineSeedsYKw', 'coarseSeedsKw', 'hybridBlocksKw', 'ratioKw',
            'ratioSequenceKw', 'stageWidthsKw', 'actionKw',
            'autoRegularKw', 'generateMeshKw', 'validateKw',
            'element2dKw', 'meshRegular2dKw', 'curvedSurfaceKw',
            'pickedFaceKw', 'pickedEdgeKw', 'pickedPlanarCoarseEdgeKw',
            'pickedDomainFacesKw',
            'pickedRegionFacesKw', 'pickedFineEdgesKw',
            'pickedCoarseEdgesKw', 'pickedCellsKw', 'pickedPathEdgesKw',
        )
        snapshot = []
        for name in names:
            keyword = getattr(form, name, None)
            if keyword is None:
                continue
            if (name.startswith('picked')
                    and name not in form._active_pick_keyword_names()):
                continue
            try:
                value = keyword.getValue()
            except Exception:
                value = '<unavailable>'
            snapshot.append((name, self._safe_value(value)))
        return tuple(snapshot)

    def processUpdates(self):
        """Native combo-box updates do not reliably call the Python keyword handle override."""
        if getattr(self, '_tm_closed', False):
            return
        self._sync_action_keyword()
        values = (self._safe_value(self.form.transitionTypeKw.getValue()),
                  self._safe_value(self.form.seedsKw.getValue()),
                  self._safe_value(self.form.fineSeedsYKw.getValue()),
                  self._safe_value(self.form.hybridBlocksKw.getValue()),
                  self._safe_value(self.form.stagesKw.getValue()),
                  self._safe_value(self.form.ratioKw.getValue()),
                  self._safe_value(self.form.ratioSequenceKw.getValue()),
                  self._safe_value(self.form.stageWidthsKw.getValue()),
                  self._safe_value(self.form.actionKw.getValue()))
        if values != getattr(self, '_last_ui_values', None):
            self._last_ui_values = values
            self._apply_type_visibility()
        self._invalidate_check_if_stale()
        self._poll_check_result()

    def onSuggest(self, sender, sel, ptr):
        """Apply only suggestions returned by the matching Check result."""
        form = self.form
        result = self._check_result
        try:
            if not isinstance(result, dict):
                self._set_check_label('Suggestions unavailable')
                self._set_check_detail('Run Check and wait for its result first.')
                return 1
            suggestions = result.get('suggestions')
            if not isinstance(suggestions, dict):
                self._set_check_label('Suggestions unavailable')
                self._set_check_detail('The matching Check result returned no suggestions.')
                return 1
            changed = []
            for keyword_name, result_key, label in (
                    ('seedsKw', 'seeds', 'Seeds'),
                    ('fineSeedsYKw', 'fineSeedsY', 'fineSeedsY')):
                if result_key not in suggestions:
                    continue
                value = suggestions.get(result_key)
                if value in (None, ''):
                    continue
                value = int(value)
                if value <= 0:
                    continue
                getattr(form, keyword_name).setValue(value)
                changed.append('%s=%d' % (label, value))
            if not changed:
                if str(result.get('status', '')).upper() == 'PASS':
                    self._set_check_label('Counts are valid')
                    self._set_check_detail(
                        'The current Seeds/Layers/Ratio already satisfy the '
                        'transition rules; nothing to change.')
                else:
                    self._set_check_label('No suggested values')
                    self._set_check_detail(
                        'The matching Check result did not provide applicable '
                        'values.')
                return 1
            self._set_check_label('Suggested values applied; Check is stale')
            self._set_check_detail(', '.join(changed) + '. Run Check again.')
        except Exception as exc:
            self._set_check_label('Suggestions failed')
            self._set_check_detail(str(exc)[:180])
        self._check_fingerprint = None
        try:
            self._main.recalc()
        except Exception:
            pass
        return 1

    def _param_fingerprint(self):
        return self._form_snapshot()

    def _invalidate_check_if_stale(self):
        if getattr(self, '_tm_closed', False):
            return
        fp = self._param_fingerprint()
        # Inputs changed: the previous FAIL verdict no longer describes the
        # current request, so Apply is unblocked and must be re-checked.
        if getattr(self, '_check_fingerprint', None) not in (None, fp):
            self.form._tm_block_apply = False
            self._check_fingerprint = None
            # Drop the cached result too: Apply-suggested-values must never
            # consume suggestions produced for different inputs.
            self._check_result = None
            self._set_check_label(
                'Check is stale')
            self._set_check_detail(
                'Inputs changed. Press Check again before executing.')
        if (getattr(self, '_check_request_id', None) is not None
                and getattr(self, '_check_snapshot', None) not in (None, fp)):
            self._set_check_label('Check is stale')
            self._set_check_detail(
                'Inputs changed while Check was running; its result will be ignored.')

    def _dispatch_count(self):
        """How many commands the form has built (plugin-side counter)."""
        try:
            return int(getattr(self.form, '_tm_dispatch_count', 0) or 0)
        except Exception:
            return 0

    def _clear_pending_check_state(self):
        """Release a pending request idempotently (no result will arrive)."""
        self._check_request_id = None
        self._check_result_file = None
        self._check_snapshot = None
        self._check_started_at = None
        self._check_kind = 'CHECK'
        self._suggest_after = False
        try:
            self.form._tm_check_running = False
        except Exception:
            pass
        try:
            self.form.requestIdKw.setValue('')
            self.form.resultFileKw.setValue('')
        except Exception:
            pass

    def shutdown(self):
        """Release pick procedures before AFX destroys the dialog's native widgets."""
        # created by IY,China University of Petroleum (East China)  |  禁止商用 / non-commercial use only
        if getattr(self, '_tm_closed', False):
            return
        self._tm_closed = True
        try:
            self.closePendingCheck()
        except Exception:
            pass
        handlers = list(getattr(self, 'pick_handlers', []) or [])
        self.pick_handlers = []
        for handler in handlers:
            try:
                handler.shutdown()
            except Exception:
                pass

    def discard(self):
        """Drop Python references to a dialog that AFX may already have destroyed."""
        self._tm_closed = True
        for handler in list(getattr(self, 'pick_handlers', []) or []):
            try:
                handler.discard()
            except Exception:
                pass
        self.pick_handlers = []

    def closePendingCheck(self):
        """Invalidates any in-flight request so a late JSON result cannot pop up a dialog or write into an"""
        self._clear_pending_check_state()
        self._check_result = None
        self._check_fingerprint = None

    @classmethod
    def _check_timeout(cls):
        try:
            return max(10.0, float(os.environ.get(cls._CHECK_TIMEOUT_ENV,
                                                 cls._CHECK_TIMEOUT_DEFAULT)))
        except Exception:
            return cls._CHECK_TIMEOUT_DEFAULT

    def onCheck(self, sender, sel, ptr):
        """Check: numeric/geometry validation only, never touches the model."""
        return self._dispatch_request('CHECK', sender, sel, ptr)

    def onPreMesh(self, sender, sel, ptr):
        """Pre-mesh: trial partition+seed+mesh, audit it, then roll back."""
        return self._dispatch_request('PRE_MESH', sender, sel, ptr)

    def onSuggestValues(self, sender, sel, ptr):
        """Suggest values: run a Check, then apply fixes or report valid."""
        return self._dispatch_request('CHECK', sender, sel, ptr,
                                      suggest_after=True)

    def onKernelException(self, exc):
        """Called by AFXForm even when command initialization/evaluation fails."""
        if getattr(self, '_tm_closed', False):
            return
        if getattr(self, '_check_dispatching', False):
            # Action/checkOnly are temporarily overridden until onCmdApply returns.
            self._check_kernel_exception = str(exc)
            return
        self._poll_check_result()
        if self._check_request_id is None:
            return
        title = 'Pre-mesh' if self._check_kind == 'PRE_MESH' else 'Check'
        self._clear_pending_check_state()
        self._set_check_label('%s failed' % title)
        self._set_check_detail('Kernel command failed: %s' % str(exc))
        # The form's base handleException posts the native error popup.

    def _dispatch_request(self, kind, sender, sel, ptr, suggest_after=False):
        """Shared async request: dispatch once, match the JSON by request id."""
        if getattr(self, '_tm_closed', False):
            return 1
        title = 'Pre-mesh' if kind == 'PRE_MESH' else 'Check'
        self._poll_check_result()
        if self._check_request_id is not None:
            active_title = ('Pre-mesh' if self._check_kind == 'PRE_MESH'
                            else 'Check')
            detail = '%s is still running. Wait for its result before starting %s.' % (
                active_title, title)
            self._set_check_label('%s is already running' % active_title)
            self._set_check_detail(detail)
            self._popup_check_result('WARN', detail, title=active_title)
            return 1
        self._suggest_after = bool(suggest_after)
        self._check_kernel_exception = None
        # Capture the selected action before Check/Pre-mesh overrides it.
        self._sync_action_keyword()
        # Sync the viewport target BEFORE taking the snapshot so the request
        # hash matches the Model/Part the kernel will infer from the picks.
        try:
            self.form._sync_viewport_target()
        except Exception:
            pass
        request_id = uuid.uuid4().hex
        result_file = os.path.join(
            tempfile.gettempdir(),
            'transition_mesh_check_%s_%s.json' % (os.getpid(), request_id))
        snapshot = self._param_fingerprint()
        dispatched_before = self._dispatch_count()
        self._check_request_id = request_id
        self._check_result_file = result_file
        self._check_snapshot = snapshot
        self._check_result = None
        self._check_fingerprint = None
        self._last_check_read_error = None
        self._check_kind = kind
        self._check_started_at = time.time()
        try:
            self.form._tm_check_running = True
        except Exception:
            pass
        try:
            self.form.requestIdKw.setValue(request_id)
            self.form.resultFileKw.setValue(result_file)
        except Exception as exc:
            self._clear_pending_check_state()
            self._set_check_label('%s dispatch failed' % title)
            self._set_check_detail(str(exc)[:180])
            return 1
        previous_check_only = False
        previous_action = None
        dispatch_error = None
        try:
            previous_check_only = bool(self.form.checkOnlyKw.getValue())
            previous_action = str(self.form.actionKw.getValue())
            self._check_dispatching = True
            try:
                self.form._tm_check_dispatching = True
            except Exception:
                pass
            if kind == 'PRE_MESH':
                # The kernel runs the real operation and rolls it back.
                self.form.actionKw.setValue('PRE_MESH')
            else:
                self.form.checkOnlyKw.setValue(True)
            # AFXDataDialog.onCmdApply is the documented action-area path.
            # The keyword is set before the command is constructed.
            self.onCmdApply(sender, sel, ptr)
        except Exception as exc:
            dispatch_error = exc
        finally:
            self._check_dispatching = False
            try:
                self.form._tm_check_dispatching = False
            except Exception:
                pass
            try:
                self.form.checkOnlyKw.setValue(previous_check_only)
            except Exception:
                pass
            if previous_action is not None:
                try:
                    self.form.actionKw.setValue(previous_action)
                except Exception:
                    pass
        # Native AFX callbacks may run synchronously inside onCmdApply.
        # Poll only after restoring the user's action so snapshots match.
        self._poll_check_result()
        kernel_error = getattr(self, '_check_kernel_exception', None)
        if kernel_error is not None:
            self._check_kernel_exception = None
            self.onKernelException(kernel_error)
        if self._check_request_id is None:
            return 1
        if dispatch_error is not None or self._dispatch_count() == dispatched_before:
            # The form refused the request (validation error) or the command
            # was never built: no JSON will arrive, so release the wait now.
            self._clear_pending_check_state()
            self._suggest_after = False
            self._set_check_label('%s not dispatched' % title)
            if dispatch_error is not None:
                detail = str(dispatch_error)[:180]
            else:
                detail = ('The form rejected the input. Fix the reported '
                          'problem, then try again.')
            self._set_check_detail(detail)
            return 1
        self._set_check_label('%s running' % title)
        self._set_check_detail(
            'Waiting for the matching kernel result; the model is read-only.'
            if kind != 'PRE_MESH' else
            'Trial-meshing now; the result is rolled back afterwards.')
        try:
            self._main.recalc()
        except Exception:
            pass
        return 1

    def _set_check_label(self, text):
        if getattr(self, '_tm_closed', False):
            return
        try:
            self.checkLabel.setText(text)
        except Exception:
            pass

    def _set_check_detail(self, text):
        if getattr(self, '_tm_closed', False):
            return
        try:
            value = str(text or '').replace('\r', ' ').replace('\n', ' | ')
            self.checkDetailLabel.setText(value[:320])
        except Exception:
            pass

    def _sync_action_combo(self):
        """Initialize the native combo keyword from the saved command action."""
        value = str(self.form.actionKw.getValue())
        for idx, item in enumerate(self._ACTION_ITEMS):
            if value == item[1]:
                self.form.actionChoiceKw.setValue(idx)
                return

    def _sync_action_keyword(self):
        """Commit the user choice, never reset it from a stale command value."""
        if self._check_dispatching:
            return  # Preserve the temporary PRE_MESH action during dispatch.
        idx = int(self.form.actionChoiceKw.getValue())
        if idx < 0 or idx >= len(self._ACTION_ITEMS):
            raise ValueError('Choose a valid Action before executing.')
        action = self._ACTION_ITEMS[idx][1]
        if str(self.form.actionKw.getValue()) != action:
            self.form.actionKw.setValue(action)
            self._invalidate_check_if_stale()

    def onHybridToggled(self, sender, sel, ptr):
        """Hybrid is single-stage by definition, so enabling it forces Layers=1;"""
        checked = False
        try:
            checked = bool(self.hybridBtn.getCheck())
        except Exception:
            checked = getattr(self, '_hybrid_on', False)
        self._hybrid_on = checked
        if checked:
            try:
                self.form.stagesKw.setValue(1)
            except Exception:
                pass
        else:
            try:
                self.form.hybridBlocksKw.setValue(0)
            except Exception:
                pass
        try:
            self._apply_type_visibility()
        except Exception:
            pass
        self._invalidate_check_if_stale()
        return 1

    def _poll_check_result(self):
        """Consume only the current request's atomically-written JSON result."""
        if getattr(self, '_tm_closed', False):
            return
        if getattr(self, '_check_dispatching', False):
            return
        request_id = getattr(self, '_check_request_id', None)
        result_file = getattr(self, '_check_result_file', None)
        if not request_id or not result_file:
            return
        # Prefer a matching completed result even if the GUI was blocked past
        # the timeout by a long kernel call. A timeout is not kernel cancellation.
        result = None
        try:
            if os.path.isfile(result_file):
                with open(result_file, 'r') as stream:
                    result = json.load(stream)
        except Exception as exc:
            self._last_check_read_error = str(exc)
        if (not isinstance(result, dict)
                or str(result.get('request_id', '')) != str(request_id)):
            started = getattr(self, '_check_started_at', None)
            if started is not None and (time.time() - started) > self._check_timeout():
                kind = self._check_kind
                title = 'Pre-mesh' if kind == 'PRE_MESH' else 'Check'
                detail = ('No matching result arrived within %d s. This does '
                          'not cancel a running kernel command. Wait until CAE '
                          'is idle before retrying.' % int(self._check_timeout()))
                if kind == 'PRE_MESH':
                    detail += ' Rollback has not been confirmed; inspect the model and repick geometry.'
                self._clear_pending_check_state()
                self._set_check_label('%s timed out' % title)
                self._set_check_detail(detail)
                self._popup_check_result('FAIL', detail, title=title)
            return
        current_snapshot = self._param_fingerprint()
        if (self._check_snapshot is not None
                and current_snapshot != self._check_snapshot):
            self._clear_pending_check_state()
            self._check_result = None
            self._set_check_label('Check result ignored as stale')
            self._set_check_detail('Inputs changed before the result arrived. Run Check again.')
            return
        status = str(result.get('status', '')).upper()
        if status not in ('PASS', 'WARN', 'FAIL'):
            status = 'FAIL'
            result['summary'] = 'Invalid Check result status.'
        kind = getattr(self, '_check_kind', 'CHECK')
        title = 'Pre-mesh' if kind == 'PRE_MESH' else 'Check'
        self._check_result = result
        self._check_fingerprint = current_snapshot
        suggest = getattr(self, '_suggest_after', False)
        self._clear_pending_check_state()
        # A FAIL Check blocks Apply until the inputs change; a PASS/WARN
        # Check or a passing Pre-mesh proves constructibility and lifts it.
        try:
            if kind == 'CHECK' and status == 'FAIL':
                self.form._tm_block_apply = True
            elif status in ('PASS', 'WARN'):
                self.form._tm_block_apply = False
        except Exception:
            pass
        pending = result.get('checks_pending') or []
        summary = result.get('summary') or 'No summary returned.'
        # Show results in the fixed bar and popup only; do not write TXT reports.
        # Optional verbose output stays in the CAE message area.
        if os.environ.get('TM_VERBOSE'):
            print(self._format_check_report(request_id, result, title))
        if pending and kind != 'PRE_MESH':
            summary = '%s Pending: %s' % (summary, ', '.join(map(str, pending)))
        self._set_check_label('%s %s' % (title, status))
        self._set_check_detail(summary)
        self._popup_check_result(status, summary, title=title)
        if suggest and kind == 'CHECK':
            # 'Suggest values' reuses this fresh result: apply fixes or say
            # the current numbers are already valid.
            self.onSuggest(None, None, None)

    @staticmethod
    def _format_check_report(request_id, result, title='Check'):
        lines = ['%s %s (request %s)' % (
            title.upper(), str(result.get('status', '')).upper(), request_id)]
        for key in ('mode', 'ratio', 'fine_seeds', 'stages', 'layers',
                    'planned_stages'):
            if result.get(key) is not None:
                lines.append('%s: %s' % (key, result.get(key)))
        for key in ('stage_counts', 'planned_stage_counts', 'summary_lines',
                    'checks_pending', 'suggestions', 'metrics', 'warnings'):
            value = result.get(key)
            if value:
                lines.append('%s: %s' % (key, value))
        lines.append('summary: %s' % (result.get('summary') or ''))
        return '\n'.join(lines)

    def _popup_check_result(self, status, summary, title='Check'):
        """Exactly one GUI notification per matched result (never stale)."""
        if getattr(self, '_tm_closed', False):
            return
        message = '%s %s\n\n%s' % (title, status, summary)
        try:
            owner = getAFXApp().getAFXMainWindow()
            show = None
            if status == 'PASS':
                show = globals().get('showAFXInformationDialog')
            elif status == 'WARN':
                show = (globals().get('showAFXWarningDialog')
                        or globals().get('showAFXInformationDialog'))
            else:
                show = globals().get('showAFXErrorDialog')
            if show is None:
                show = globals().get('showAFXErrorDialog')
            if show is not None:
                show(owner, message)
        except Exception:
            pass

    def _on_type_changed_handle(self, sender, sel, ptr):
        """Message-map fallback (used only if tm_listener is unavailable)."""
        self._apply_type_visibility()

    def create(self):
        """Let AFX build the dialog; deferred onCmdFit sizes it to content."""
        AFXDataDialog.create(self)

    def onCmdFit(self, sender, sel, ptr):
        """FXScrollWindow does not propagate its content height to the top-level dialog,"""
        try:
            screen = self.getScreen()
            screen_w = int(screen.getWidth()) if screen is not None else 1920
            screen_h = int(screen.getHeight()) if screen is not None else 1080
            max_w = max(640, int(screen_w * 0.92))
            max_h = max(520, int(screen_h * 0.9))
            try:
                content_h = int(self.mainScroll.getContentHeight())
            except Exception:
                content_h = 0
            try:
                bar_h = int(self._bottomBar.getHeight())
            except Exception:
                bar_h = 90
            try:
                default_w = int(self.getDefaultWidth())
            except Exception:
                default_w = 0
            width = min(max(default_w, 620), max_w)
            if content_h > 0:
                body_cap = max(320, int(screen_h * 0.62))
                body_h = min(content_h, body_cap)
                height = min(body_h + bar_h + 120, max_h)
            else:
                try:
                    default_h = int(self.getDefaultHeight())
                except Exception:
                    default_h = 0
                height = min(max(default_h, 600), max_h)
            x = max(0, int((screen_w - width) / 2))
            y = max(0, int((screen_h - height) / 2))
            self.position(x, y, width, height)
        except Exception:
            pass
        return 1

    def _apply_type_visibility(self):
        """Show only the groups that belong to the current Type."""
        ttype = _canonical_type(self.form.transitionTypeKw.getValue())
        if ttype is None:
            ttype = 'PLANAR'
        plan = {
            # show_planar_sel, show_p2d_sel, show_cyl_sel,
            # and separately params below.
            'PLANAR': (True, False, False),
            'PLANAR_ANNULAR': (False, True, False),
            'PLANAR_QUAD_FOUR': (False, True, False),
            'ANNULAR_RADIAL': (False, False, True),
            'AXIAL_SURFACE': (False, False, True),
        }[ttype]
        show_planar, show_p2d, show_cyl = plan
        groups = (
            (self.selPlanar, show_planar), (self.selP2D, show_p2d),
            (self.selCyl, show_cyl),
            (self.paramPlanar, show_planar),
            (self.paramCyl, show_cyl),
        )
        for widget, visible in groups:
            try:
                if visible:
                    widget.show()
                else:
                    widget.hide()
            except Exception:
                pass
        # Ny sits next to Nx and belongs to PLANAR_QUAD_FOUR only; every other
        # type hides it (the kernel treats a hidden Ny=0 as "same as Nx").
        ny_visible = (ttype == 'PLANAR_QUAD_FOUR')
        for widget in (getattr(self, 'nyField', None),):
            if widget is None:
                continue
            try:
                if ny_visible:
                    widget.show()
                else:
                    widget.hide()
            except Exception:
                pass
        # Classic PLANAR domain pick and optional coarse edge live in selPlanar;
        # regular-region meshing is driven by whether the domain pick is non-empty (no checkbox).
        planar_domain = (ttype == 'PLANAR')
        for widget in (getattr(self, 'planarDomainPick', None),
                       getattr(self, 'planarCoarsePick', None)):
            if widget is None:
                continue
            try:
                if planar_domain:
                    widget.show()
                else:
                    widget.hide()
            except Exception:
                pass
        # Hybrid switch + 2:1 block field belong to classic PLANAR only;
        # the block field additionally needs the switch to be on.
        hybrid_on = planar_domain and bool(getattr(self, '_hybrid_on', False))
        for widget, visible in (
                (getattr(self, 'hybridBtn', None), planar_domain),
                (getattr(self, 'hybridBlocksField', None), hybrid_on)):
            if widget is None:
                continue
            try:
                if visible:
                    widget.show()
                else:
                    widget.hide()
            except Exception:
                pass
        for widget in (getattr(self, 'layersField', None),
                       getattr(self, 'ratioCombo', None)):
            if widget is None:
                continue
            try:
                if hybrid_on:
                    widget.hide()
                else:
                    widget.show()
            except Exception:
                pass
        # shared seeds / ratio / action group is used by every type
        try:
            self.paramShared.show()
        except Exception:
            pass
        try:
            self._main.recalc()
        except Exception:
            pass
        # seeds semantics label per type
        seeds_text = {
            'PLANAR': 'Seeds N (fine edge):',
            'PLANAR_ANNULAR': 'Fine loop N:',
            'PLANAR_QUAD_FOUR': 'Nx (U-side):',
            'ANNULAR_RADIAL': 'Fine loop N:',
            'AXIAL_SURFACE': 'Seeds N (per section):',
        }[ttype]
        try:
            self.seedsLabel.setText(seeds_text)
        except Exception:
            pass
        # Re-label the shared group so the current mode is explicit.
        try:
            self.paramShared.setText('Transition - %s' % ttype)
        except Exception:
            pass
        self._update_computed_info(ttype)

    def _capability_summary(self, ttype):
        """Returns (capability_dict, extra_note) or (None,"""
        try:
            import transition_mesh_rules as tm_rules
        except Exception:
            return None, None
        form = self.form
        try:
            # @author  created by IY,China University of Petroleum (East China)  --  禁止商用 (non-commercial)
            layers = int(form.stagesKw.getValue() or 1)
        except Exception:
            layers = 1
        ratio = str(form.ratioKw.getValue() or '2:1').strip()
        try:
            seeds = int(form.seedsKw.getValue() or 0)
        except Exception:
            seeds = 0
        try:
            coarse = int(form.coarseSeedsKw.getValue() or 0)
        except Exception:
            coarse = 0
        try:
            blocks = int(form.hybridBlocksKw.getValue() or 0)
        except Exception:
            blocks = 0
        hybrid_on = (ttype == 'PLANAR'
                     and bool(getattr(self, '_hybrid_on', False)))
        if hybrid_on and blocks > 0 and seeds > 0 and (seeds + blocks) % 3 == 0:
            coarse = (seeds + blocks) // 3
        hybrid = (hybrid_on or coarse > 0)
        quad = (ttype == 'PLANAR_QUAD_FOUR')
        try:
            cap = tm_rules.capability(ttype, ratio, layers, hybrid=hybrid,
                                      seeds=seeds, coarse_seeds=coarse)
        except Exception:
            return None, None
        extra = ''
        try:
            if seeds > 0 and not hybrid and cap['status'] != 'no':
                ok, _err = tm_rules.counts_ok(seeds, ratio, layers, quad=quad)
                if not ok:
                    need = tm_rules.required_multiple(
                        int(str(ratio).split(':')[0]), layers, quad)
                    cands = tm_rules.nearby_valid_counts(
                        seeds, ratio, layers, quad=quad)
                    extra = '  |  N=%d needs a multiple of %d; try %s' % (
                        seeds, need, '/'.join(str(c) for c in cands))
        except Exception:
            extra = ''
        if cap['status'] == 'warn':
            body = cap['note'] or cap['reason']
        elif cap['status'] == 'no':
            body = ' '.join([x for x in (cap['reason'], cap['fix']) if x])
        else:
            body = cap['precondition']
        text = cap['headline']
        if body:
            text = '%s - %s' % (text, body)
        text += extra
        return cap, text[:230]

    def _update_computed_info(self, ttype):
        """Summarise the layer/ratio choice; the size maths lives in Check."""
        if getattr(self, '_tm_closed', False):
            return
        form = self.form
        try:
            layers = int(form.stagesKw.getValue())
            ratio = str(form.ratioKw.getValue() or '2:1').strip()
            text = ('Layers=%d x %s. Check converts the fine size to the '
                    'target coarse size and validates the counts.'
                    % (layers, ratio))
        except Exception:
            text = 'Enter Layers and Ratio, then run Check.'
        try:
            self.sharedInfoLabel.setText(text)
        except Exception:
            pass
        # Capability line + field locking from the shared rules table.
        cap, cap_text = self._capability_summary(ttype)
        try:
            if cap_text:
                self.capabilityLabel.setText(cap_text)
            elif cap is None:
                self.capabilityLabel.setText(
                    'Capability: rules module not loaded (Check still applies '
                    'the kernel rules).')
        except Exception:
            pass
        try:
            # Hybrid cannot be combined with Layers > 1: lock the field so an
            # impossible request cannot be typed in.
            if cap is not None and cap['status'] == 'no':
                self.layersField.disable()
            else:
                self.layersField.enable()
        except Exception:
            pass
        # The PLANAR 2D regular-region hint was removed: meshing is now
        # implied by the Full Domain pick, so nothing extra to say here.

    def _add_pick_button(self, parent, labelText, keyword, prompt,
                         entitiesToPick, numberToPick=_PICK.MANY):
        pickHf = FXHorizontalFrame(
            p=parent, opts=LAYOUT_FILL_X, x=0, y=0, w=0, h=0,
            pl=0, pr=0, pt=0, pb=0, hs=DEFAULT_SPACING, vs=DEFAULT_SPACING
        )
        pickHf.setSelector(99)
        label = FXLabel(
            p=pickHf, text='    ' + labelText + ' (None)', ic=None,
            opts=LAYOUT_CENTER_Y | JUSTIFY_LEFT
        )
        pickHandler = TransitionMeshDBPickHandler(
            form=self.form,
            keyword=keyword,
            prompt=prompt,
            entitiesToPick=entitiesToPick,
            numberToPick=numberToPick,
            label=label
        )
        self.pick_handlers.append(pickHandler)
        icon = afxGetIcon('select', AFX_ICON_SMALL)
        FXButton(
            p=pickHf, text='\tPick Items in Viewport', ic=icon,
            tgt=pickHandler, sel=AFXMode.ID_ACTIVATE,
            opts=BUTTON_NORMAL | LAYOUT_CENTER_Y, x=0, y=0, w=0, h=0,
            pl=2, pr=2, pt=1, pb=1
        )
        # The AFXForm outlives an individual dialog.
        # Reflect the real keyword immediately instead of always claiming "None" when a selection is still
        pickHandler.refreshLabel()


class TransitionMeshDBPickHandler(AFXProcedure):

    count = 0

    def __init__(self, form, keyword, prompt, entitiesToPick, numberToPick, label):
        self.form = form
        self.keyword = keyword
        self.prompt = prompt
        self.entitiesToPick = entitiesToPick
        self.numberToPick = numberToPick
        self.label = label
        self.labelText = label.getText()
        self._saved_value = None  # 保存拾取前的合法值
        self._shutdown = False

        AFXProcedure.__init__(self, form.getOwner())

        TransitionMeshDBPickHandler.count += 1
        self.setModeName('TransitionMeshDBPickHandler%d' % TransitionMeshDBPickHandler.count)

    def activate(self):
        """Pick 开始前保存当前合法值，清除上一次临时高亮。"""
        if getattr(self, '_shutdown', False):
            return
        val = self.keyword.getValue()
        if val and not str(val).strip().startswith('<'):
            self._saved_value = val
        else:
            self._saved_value = None
        self._clear_pick_visuals()
        AFXProcedure.activate(self)

    def _clear_pick_visuals(self):
        """Release the viewport-side temporary pick/highlight state."""
        try:
            sendCommand('session.viewports[session.currentViewportName].'
                        'view.setValues()')
        except Exception:
            pass

    def onCancel(self):
        """Cancel must restore the last complete selection, not a partial pick."""
        if getattr(self, '_shutdown', False):
            return None
        try:
            if self._saved_value is None:
                self.keyword.setValue('')
            else:
                self.keyword.setValue(self._saved_value)
        except Exception:
            pass
        self._clear_pick_visuals()
        try:
            return AFXProcedure.onCancel(self)
        except Exception:
            return None

    def getFirstStep(self):
        return AFXPickStep(
            self, self.keyword, self.prompt,
            self.entitiesToPick, self.numberToPick,
            sequenceStyle=_PICK.TUPLE
        )

    def getNextStep(self, previousStep):
        """拾取完成后更新标签显示实际数量，不再自动高亮。"""
        self.refreshLabel()
        return None

    def deactivate(self):
        """退出拾取状态：保存值、更新计数、不再次自动高亮。"""
        if getattr(self, '_shutdown', False):
            return
        AFXProcedure.deactivate(self)
        self._clear_pick_visuals()
        # 不调用 highlight——避免持续高亮干扰用户下一步操作
        self.refreshLabel()

    def shutdown(self):
        """Deactivate while the dialog still owns native widgets, then drop refs."""
        if getattr(self, '_shutdown', False):
            return
        try:
            self.deactivate()
        except Exception:
            pass
        self.discard()

    def discard(self):
        """Forget a dialog that may already have been destroyed by AFX."""
        self._shutdown = True
        self.label = None
        self.labelText = ''
        self.keyword = None
        self.form = None
        self._saved_value = None

    def refreshLabel(self):
        """Synchronize the visible count with the actual AFX keyword."""
        if (getattr(self, '_shutdown', False)
                or getattr(self, 'label', None) is None
                or getattr(self, 'keyword', None) is None):
            return 0
        val = self.keyword.getValue()
        n = self._count_picked(val)
        if n > 0:
            base = self.labelText.split(' (')[0]
            self.label.setText('%s (%d picked)' % (base, n))
        else:
            self.label.setText(self.labelText)
        return n

    def _count_picked(self, val):
        """统计拾取结果中的实体数量。"""
        if val is None:
            return 0
        text = str(val).strip()
        if text == '' or text.startswith('<'):
            return 0
        # 尝试解析为序列计算长度
        try:
            namespace = {'mdb': mdb}
            resolved = eval(text, namespace) if isinstance(val, str) else val
            if hasattr(resolved, '__len__'):
                return len(resolved)
            if hasattr(resolved, '__iter__'):
                return sum(1 for _ in resolved)
            return 1
        except Exception:
            # 字符串中含 findAt 则至少有一个实体
            if 'findAt' in text:
                return text.count('findAt')
            return 1 if text else 0
