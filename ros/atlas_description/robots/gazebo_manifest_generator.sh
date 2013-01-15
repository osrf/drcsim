#!/bin/bash
cat <<END
<?xml version='1.0'?>
<model>
  <name>$1</name>
  <description>$1</description>
  <version>0.2.0</version>
  <sdf>$1.urdf</sdf>
  <author>
    <name>John Hsu</name>
    <email>hsu@osrfoundation.org</email>
  </author>
  <description>
    This model approximates the Atlas robot built by Boston Dynamics.
  </description>
</model>
END

