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

  def testUnknownOptionsAreRefused(self):
    proc = subprocess.run([self.cmd, '-I', 'nonexistent', '--bogus'],
                           stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                           stderr=subprocess.STDOUT)
    self.assertNotEqual(proc.returncode, 0)
    self.assertIn('unrecognized', proc.stdout.decode())

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
