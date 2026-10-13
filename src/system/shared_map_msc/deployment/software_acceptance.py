"""Run named software scenarios and emit machine-readable evidence, explicitly not physical acceptance."""
import argparse,io,json,time,unittest,sys
from pathlib import Path
parser=argparse.ArgumentParser();parser.add_argument('--tests',required=True);parser.add_argument('--output',required=True);args=parser.parse_args()
directory=Path(args.tests).resolve();sys.path.insert(0,str(directory));suite=unittest.TestSuite()
for name in ('core','traffic','tasks','task_integration','planning','maneuver','metrics','arc_safety','scenarios'):
 suite.addTests(unittest.defaultTestLoader.discover(str(directory),pattern='test_'+name+'.py'))
stream=io.StringIO();started=time.monotonic();result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
report={'kind':'software_state_and_geometry_regression','physical_motion_tested':False,'tests':result.testsRun,
        'passed':result.testsRun-len(result.failures)-len(result.errors)-len(result.skipped),'seconds':time.monotonic()-started,
        'failures':[(str(t),trace) for t,trace in result.failures+result.errors],'log':stream.getvalue()}
Path(args.output).write_text(json.dumps(report,indent=2,ensure_ascii=False))
print(stream.getvalue());print(json.dumps({k:v for k,v in report.items() if k!='log'},ensure_ascii=False))
raise SystemExit(0 if result.wasSuccessful() and result.testsRun>=40 else 1)
