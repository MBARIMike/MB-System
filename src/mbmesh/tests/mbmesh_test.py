#!/usr/bin/env python3

"""Smoke test for the mbmesh command line app itself (as opposed to the
algorithm-level tests alongside this file, e.g. test_settings.cpp)."""

import subprocess
import unittest


class MbmeshTest(unittest.TestCase):

  def setUp(self):
    self.cmd = './mbmesh'

  def testHelp(self):
    cmd = [self.cmd, '-h']
    output = subprocess.check_output(cmd, stderr=subprocess.STDOUT).decode()
    self.assertIn('mbmesh generates 3D meshes from swath sonar bathymetry',
                   output)
    self.assertIn('usage: mbmesh', output)

  def testGeoOriginRequiresThreeValues(self):
    # Same rule as mbgrd2gltf: a specified GeoOrigin must be lon,lat,elev.
    for arg in ('--geoorigin=-122.4,36.6', '-G-122.4'):
      proc = subprocess.run([self.cmd, '-I', 'nonexistent', arg],
                             stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, check=False)
      self.assertNotEqual(proc.returncode, 0, arg)
      self.assertIn('GeoOrigin format must be lon,lat,elev',
                    proc.stdout.decode(), arg)

  def testChunkOptionsAreValidated(self):
    proc = subprocess.run([self.cmd, '-I', 'nonexistent', '--chunk=abc'],
                           stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, check=False)
    self.assertNotEqual(proc.returncode, 0)
    self.assertIn('invalid --chunk argument', proc.stdout.decode())

  def testPoissonSolverOptionsAreValidated(self):
    # an unknown solver or a bad count is refused before any data is read
    for arg, message in (('--poisson-solver=bogus', "'bogus' is not one of jacobi, sor"),
                         ('--poisson-iterations=0', 'whole number from 1 to 100000'),
                         ('--poisson-iterations=x', 'whole number from 1 to 100000')):
      proc = subprocess.run([self.cmd, '-I', 'nonexistent', arg],
                             stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, check=False)
      self.assertNotEqual(proc.returncode, 0, arg)
      output = proc.stdout.decode()
      self.assertIn(message, output, arg)
      self.assertNotIn('reading datalist', output, arg)

  def testPoissonSolverNamesAreAccepted(self):
    # a known solver gets past option parsing, to the missing input
    for name in ('jacobi', 'sor'):
      proc = subprocess.run([self.cmd, '-I', 'nonexistent', '--poisson-solver=' + name],
                             stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, check=False)
      self.assertNotIn('invalid --poisson-solver', proc.stdout.decode(), name)

  def testHelpDocumentsPoissonSolver(self):
    output = subprocess.check_output([self.cmd, '-h'],
                                      stderr=subprocess.STDOUT).decode()
    for option in ('--poisson-solver', '--poisson-iterations'):
      self.assertIn(option, output)

  def testHelpDocumentsChunking(self):
    output = subprocess.check_output([self.cmd, '-h'],
                                      stderr=subprocess.STDOUT).decode()
    for option in ('--chunk-size', '--chunk <i,j>', '--max-chunks', '--threads'):
      self.assertIn(option, output)

  def testInfoOptionIsDocumented(self):
    output = subprocess.check_output([self.cmd, '-h'],
                                      stderr=subprocess.STDOUT).decode()
    self.assertIn('--info', output)
    self.assertNotIn('--metadata', output)

  def testOldMetadataOptionIsRefusedNotIgnored(self):
    # --metadata was renamed --info. It must not be silently ignored, which
    # would start a full reconstruction.
    for option in ('--metadata', '-metadata'):
      proc = subprocess.run([self.cmd, '-I', 'nonexistent', option],
                             stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, check=False)
      self.assertNotEqual(proc.returncode, 0, option)
      output = proc.stdout.decode()
      self.assertIn('renamed --info', output, option)
      # it stopped before reading any data
      self.assertNotIn('reading datalist', output, option)

  def testGeoOriginValueMustBeAttached(self):
    # "--geoorigin VALUE" is the option with no value plus a stray argument, which
    # used to be ignored and gave the automatic GeoOrigin without a word.
    for value in ('122.4,36.6,0', '-122.4,36.6,0'):
      proc = subprocess.run([self.cmd, '-I', 'nonexistent', '--geoorigin', value],
                             stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, check=False)
      self.assertNotEqual(proc.returncode, 0, value)
      output = proc.stdout.decode()
      self.assertNotIn('reading datalist', output, value)
      self.assertTrue('unexpected argument' in output or 'unrecognized' in output, value)

  def testUnknownOptionsAreRefused(self):
    proc = subprocess.run([self.cmd, '-I', 'nonexistent', '--bogus'],
                           stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, check=False)
    self.assertNotEqual(proc.returncode, 0)
    self.assertIn('unrecognized', proc.stdout.decode())

  def testGeoViewpointIsValidated(self):
    # --geoviewpoint takes 6 or 10 numbers and needs --html; bad values are refused
    # before any data are read.
    cases = (
      (['--html', '--geoviewpoint=1,2,3,4,5'], '6 numbers'),
      (['--html', '--geoviewpoint=1,2,3,4,5,x'], "'x'"),
      (['--html', '--geoviewpoint=1 2 3 4 5 6'], 'not a number'),
      (['--geoviewpoint=1,2,3,4,5,6'], 'use --html too'),
    )
    for args, message in cases:
      proc = subprocess.run([self.cmd, '-I', 'nonexistent'] + args,
                             stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, check=False)
      self.assertNotEqual(proc.returncode, 0, args)
      self.assertIn(message, proc.stdout.decode(), args)
      self.assertNotIn('reading datalist', proc.stdout.decode(), args)

  def testLiftedIsValidated(self):
    # --lifted takes meters above zero and needs --html or a directory input.
    cases = (
      (['--html', '--lifted=abc'], 'not a number of meters'),
      (['--html', '--lifted=-5'], 'not a number of meters'),
      (['--html', '--lifted=0'], 'not a number of meters'),
      (['--lifted=100'], 'use --html too'),
    )
    for args, message in cases:
      proc = subprocess.run([self.cmd, '-I', 'nonexistent'] + args,
                             stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, check=False)
      self.assertNotEqual(proc.returncode, 0, args)
      self.assertIn(message, proc.stdout.decode(), args)

  def testSpillPointsIsValidated(self):
    # --spill-points takes a whole number of soundings, zero or more.
    for value in ('abc', '-5', '1.5', ''):
      proc = subprocess.run([self.cmd, '-I', 'nonexistent', '--spill-points=' + value],
                             stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, check=False)
      self.assertNotEqual(proc.returncode, 0, value)
      self.assertIn('not a whole number of soundings', proc.stdout.decode(), value)

  def testHelpDocumentsSpillPoints(self):
    output = subprocess.check_output([self.cmd, '-h'],
                                      stderr=subprocess.STDOUT).decode()
    self.assertIn('--spill-points', output)
    self.assertIn('mbmesh_work', output)

  def testPointcloudColormapIsValidated(self):
    proc = subprocess.run([self.cmd, '-I', 'nonexistent', '--pointcloud-colormap=rainbow'],
                           stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT, check=False)
    self.assertNotEqual(proc.returncode, 0)
    self.assertIn('linear, equalized, none', proc.stdout.decode())

  def testHelpDocumentsPointcloudColormap(self):
    output = subprocess.check_output([self.cmd, '-h'],
                                      stderr=subprocess.STDOUT).decode()
    self.assertIn('--pointcloud-colormap', output)

  def testHelpDocumentsLifted(self):
    output = subprocess.check_output([self.cmd, '-h'],
                                      stderr=subprocess.STDOUT).decode()
    self.assertIn('--lifted', output)

  def testHelpDocumentsGeoViewpoint(self):
    output = subprocess.check_output([self.cmd, '-h'],
                                      stderr=subprocess.STDOUT).decode()
    self.assertIn('--geoviewpoint', output)
    self.assertIn('Copy GeoViewpoint', output)

  def testHelpDocumentsTileset(self):
    output = subprocess.check_output([self.cmd, '-h'],
                                      stderr=subprocess.STDOUT).decode()
    self.assertIn('--tileset', output)
    self.assertIn('tileset/tileset.json', output)
    self.assertIn('--x3dom-url', output)

  def testHelpDocumentsGeoOrigin(self):
    output = subprocess.check_output([self.cmd, '-h'],
                                      stderr=subprocess.STDOUT).decode()
    self.assertIn('--geoorigin', output)

  def testOutputGoesBesideTheDatalistAndDirectoryInputWritesSurveyPage(self):
    import os, struct, tempfile
    cmd = os.path.abspath(self.cmd)
    with tempfile.TemporaryDirectory() as work:
      survey = os.path.join(work, '20260101A')
      os.makedirs(os.path.join(survey, 'multibeam', 'mbmesh_output', 'tiles'))
      os.makedirs(os.path.join(survey, 'lidar'))
      for name in ('multibeam/mbmesh_output/mesh.glb', 'multibeam/mbmesh_output/tiles/t.glb', 'lidar/l.glb'):
        with open(os.path.join(survey, name), 'wb') as f:
          f.write(b'glTF' + struct.pack('<III', 2, 28, 0))
      # Without -O the output directory is beside the datalist
      proc = subprocess.run([cmd, '-I', 'multibeam/datalistp.mb-1'],
                             stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, cwd=survey, check=False)
      self.assertIn('output=multibeam/mbmesh_output', proc.stdout.decode())
      proc = subprocess.run([cmd, '-I', 'multibeam/datalistp.mb-1', '-O', 'elsewhere'],
                             stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, cwd=survey, check=False)
      self.assertIn('output=elsewhere', proc.stdout.decode())
      # A directory input writes only the survey page, in the current directory
      proc = subprocess.run([cmd, '-I', '20260101A'],
                             stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, cwd=work, check=False)
      self.assertEqual(proc.returncode, 0, proc.stdout.decode())
      with open(os.path.join(work, 'mbmesh_survey.html')) as f:
        page = f.read()
      self.assertIn('<summary><b>multibeam</b></summary>', page)
      self.assertIn("uri: '20260101A/lidar/l.glb'", page)
      self.assertIn("uri: '20260101A/multibeam/mbmesh_output/mesh.glb'", page)
      self.assertNotIn('t.glb', page)

  def testNoInputFailsCleanly(self):
    # With no arguments at all the usage is shown, as mbgrd2gltf does
    cmd = [self.cmd]
    proc = subprocess.run(cmd, stdin=subprocess.DEVNULL,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False)
    self.assertEqual(proc.returncode, 0)
    output = proc.stdout.decode()
    self.assertIn('mbmesh generates 3D meshes from swath sonar bathymetry', output)
    self.assertIn('usage: mbmesh', output)
    self.assertIn('Earth-Centered, Earth-Fixed', output)
    # Options but no input is still an error
    proc = subprocess.run([self.cmd, '-V'], stdin=subprocess.DEVNULL,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False)
    self.assertNotEqual(proc.returncode, 0)
    self.assertIn('input datalist path is empty', proc.stdout.decode())


if __name__ == '__main__':
  unittest.main()
