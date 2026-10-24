#!/usr/bin/env python3

"""Tests for mbgrd2gltf command line app."""

import subprocess
import unittest


class Mbgrd2gltfTest(unittest.TestCase):

  def setUp(self):
    self.cmd = '../../src/mbgrd2gltf/mbgrd2gltf'

  def testHelp(self):
    cmd = [self.cmd, '-h']
    output = subprocess.check_output(cmd, stderr=subprocess.STDOUT).decode()
    self.assertIn('MBgrd2gltf', output)
    self.assertIn('converts a GMT GRD format bathymetry grid file', output)
    self.assertIn('usage: mbgrd2gltf', output)

  def testGeoViewpointIsValidated(self):
    # --geoviewpoint takes 6 or 10 numbers, attached, and needs --html.
    cases = (
      (['--html', '--geoviewpoint=1,2,3'], 'takes 6 numbers'),
      (['--html', '--geoviewpoint=a,b,c,d,e,f'], 'is not a number'),
      (['--html', '--geoviewpoint=36.6,-122.4,-900,36.61,-122.39,-950'], 'lon,lat,elev'),
      (['--geoviewpoint=-122.4,36.6,-900,-122.39,36.61,-950'], 'use --html too'),
    )
    for args, message in cases:
      proc = subprocess.run([self.cmd, '-I', 'nonexistent.grd'] + args,
                            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT)
      self.assertNotEqual(proc.returncode, 0, args)
      self.assertIn(message, proc.stdout.decode(), args)

  def testHelpDocumentsGeoViewpoint(self):
    output = subprocess.check_output([self.cmd, '-h'],
                                     stderr=subprocess.STDOUT).decode()
    self.assertIn('--geoviewpoint', output)
    self.assertIn('Copy GeoViewpoint', output)


if __name__ == '__main__':
  unittest.main()
