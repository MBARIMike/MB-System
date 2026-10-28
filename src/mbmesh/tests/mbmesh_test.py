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
                             stderr=subprocess.STDOUT)
      self.assertNotEqual(proc.returncode, 0, arg)
      self.assertIn('GeoOrigin format must be lon,lat,elev',
                    proc.stdout.decode(), arg)

  def testChunkOptionsAreValidated(self):
    proc = subprocess.run([self.cmd, '-I', 'nonexistent', '--chunk=abc'],
                           stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT)
    self.assertNotEqual(proc.returncode, 0)
    self.assertIn('invalid --chunk argument', proc.stdout.decode())

  def testPoissonSolverOptionsAreValidated(self):
    # an unknown solver or a bad count is refused before any data is read
    for arg, message in (('--poisson-solver=bogus', "'bogus' is not one of jacobi, sor"),
                         ('--poisson-iterations=0', 'whole number from 1 to 100000'),
                         ('--poisson-iterations=x', 'whole number from 1 to 100000')):
      proc = subprocess.run([self.cmd, '-I', 'nonexistent', arg],
                             stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT)
      self.assertNotEqual(proc.returncode, 0, arg)
      output = proc.stdout.decode()
      self.assertIn(message, output, arg)
      self.assertNotIn('reading datalist', output, arg)

  def testPoissonSolverNamesAreAccepted(self):
    # a known solver gets past option parsing, to the missing input
    for name in ('jacobi', 'sor'):
      proc = subprocess.run([self.cmd, '-I', 'nonexistent', '--poisson-solver=' + name],
                             stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT)
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
                             stderr=subprocess.STDOUT)
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
                             stderr=subprocess.STDOUT)
      self.assertNotEqual(proc.returncode, 0, value)
      output = proc.stdout.decode()
      self.assertNotIn('reading datalist', output, value)
      self.assertTrue('unexpected argument' in output or 'unrecognized' in output, value)

  def testUnknownOptionsAreRefused(self):
    proc = subprocess.run([self.cmd, '-I', 'nonexistent', '--bogus'],
                           stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT)
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
                             stderr=subprocess.STDOUT)
      self.assertNotEqual(proc.returncode, 0, args)
      self.assertIn(message, proc.stdout.decode(), args)
      self.assertNotIn('reading datalist', proc.stdout.decode(), args)

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

  def testNoInputFailsCleanly(self):
    cmd = [self.cmd]
    proc = subprocess.run(cmd, stdin=subprocess.DEVNULL,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    self.assertNotEqual(proc.returncode, 0)
    output = proc.stdout.decode()
    self.assertIn('input datalist path is empty', output)


if __name__ == '__main__':
  unittest.main()
