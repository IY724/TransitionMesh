# -*- coding: utf-8 -*-
# ============================================================================
#  Transition Mesh
#  created by IY,China University of Petroleum (East China)
#  禁止商用 / Non-commercial use only
#  禁止商用：本软件仅供学习与研究使用，未经作者书面许可不得用于任何商业用途。
#  Non-commercial use only. Unauthorized commercial use is prohibited.
# ============================================================================

"""Request reporting and one-owner model transactions for the CAE entry point."""
import json
import os
import tempfile
import time
import traceback
import uuid

_active = None


def _log_path():
    folder = os.path.join(tempfile.gettempdir(), 'TransitionMesh', 'logs')
    os.makedirs(folder, exist_ok=True)
    return folder


def stage(name, **details):
    if _active is not None:
        record = dict(details, stage=name, time=time.time(),
                      operation_id=_active.operation_id)
        with open(_active.log_file, 'a', encoding='utf-8') as stream:
            stream.write(json.dumps(record, default=str, ensure_ascii=True) + '\n')
            # created by IY,China University of Petroleum (East China)
            stream.flush()


def transaction_active():
    """Adapters that would otherwise open their own transaction must delegate into the owner instead of"""
    return _active is not None


def owned_backup(part):
    if _active is None:
        return part.backup()
    stage('INNER_BACKUP_DELEGATED', part=part.name)


def owned_restore(part):
    if _active is None:
        return part.restore()
    stage('INNER_RESTORE_DELEGATED', part=part.name)


def run_isolated_cae(script_path, workdir, timeout_sec=600, env=None):
    """Native ABORT cannot be caught in-process."""
    import subprocess
    script_path = os.path.abspath(script_path)
    workdir = os.path.abspath(workdir or os.path.dirname(script_path))
    if not os.path.isfile(script_path):
        raise ValueError('Isolated script not found: %s' % script_path)
    stage('ISOLATE_START', script=script_path, workdir=workdir)
    cmd = ['abaqus', 'cae', 'noGUI=' + script_path]
    try:
        proc = subprocess.Popen(
            cmd, cwd=workdir, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            env=env, universal_newlines=True)
        try:
            out, err = proc.communicate(timeout=timeout_sec)
            timed_out = False
        except Exception:
            proc.kill()
            out, err = proc.communicate()
            timed_out = True
        rec = {
            'returncode': proc.returncode,
            'stdout': (out or '')[-4000:],
            'stderr': (err or '')[-4000:],
            'timed_out': timed_out,
            'ok': (not timed_out) and proc.returncode == 0,
        }
        stage('ISOLATE_END', **rec)
        return rec
    except Exception as exc:
        rec = {'returncode': -1, 'stdout': '', 'stderr': str(exc),
               'timed_out': False, 'ok': False}
        stage('ISOLATE_ERROR', **rec)
        return rec


def publish(result_file, request_id, payload):
    # created by IY,China University of Petroleum (East China)  |  禁止商用 / non-commercial use only
    result = dict(payload)
    result['request_id'] = request_id
    if result_file:
        filename = os.path.abspath(result_file)
        os.makedirs(os.path.dirname(filename), exist_ok=True)
        tmpname = filename + '.' + uuid.uuid4().hex + '.tmp'
        with open(tmpname, 'w', encoding='utf-8') as stream:
            json.dump(result, stream, default=str, ensure_ascii=True)
            stream.flush()
        os.replace(tmpname, filename)
    print('CHECK %s: %s' % (result.get('status', 'FAIL'), result.get('summary', '')))
    return result


class ModelTransaction:
    def __init__(self, mdb, model_name, operation_id=None):
        self.mdb = mdb
        self.model_name = model_name
        self.operation_id = operation_id or uuid.uuid4().hex
        self.backup_name = '__TM_BACKUP_' + self.operation_id[:12]
        self.log_file = os.path.join(_log_path(), self.operation_id + '.jsonl')
        self.snapshot = False

    def __enter__(self):
        global _active
        if _active is not None:
            raise RuntimeError('TM-BUSY: another transition operation is running.')
        _active = self
        try:
            stage('SNAPSHOT_START', model=self.model_name)
            self.mdb.Model(name=self.backup_name,
                           objectToCopy=self.mdb.models[self.model_name])
            self.snapshot = True
            stage('SNAPSHOT_READY')
            return self
        except BaseException:
            _active = None
            raise

    def __exit__(self, error_type, error, tb):
        global _active
        try:
            if error_type is not None:
                stage('FAILED', error=str(error), traceback=''.join(
                    traceback.format_exception(error_type, error, tb)))
                if self.snapshot:
                    # Restore the complete model; old picked entities must be
                    # discarded by the caller after this failure.
                    stage('RESTORE_START')
                    del self.mdb.models[self.model_name]
                    self.mdb.models.changeKey(fromName=self.backup_name,
                                              toName=self.model_name)
                    stage('RESTORE_COMPLETE', repick_required=True)
                    try:
                        from abaqus import session
                        restored = self.mdb.models[self.model_name]
                        for vp in session.viewports.values():
                            shown = vp.displayedObject
                            pname = getattr(shown, 'name', '')
                            if pname in restored.parts:
                                vp.setValues(displayedObject=restored.parts[pname])
                    except Exception:
                        # A viewport refresh is optional and must not hide
                        # the original operation error or affect recovery.
                        pass
            elif self.snapshot:
                stage('COMMIT')
                del self.mdb.models[self.backup_name]
        finally:
            _active = None
        return False
