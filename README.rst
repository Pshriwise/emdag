=======
 EmDAG
=======

This package is designed to replace MOAB's ray tracing kernel in DAGMC with `Embree <embree.github.io>`, a high performanc CPU-base ray tracer developed by Intel.

==============
 Installation
==============

First install Embree and add its install location to your PATH environment variable:

.. code:: bash
	  
	  export PATH=<your_embree_install_location>:$PATH

Then go about installing DAGMC as per the online `instructions <http://svalinn.github.io/DAGMC/usersguide/get_install.html>`. However, before the final step of installing DAGMC do the following:


.. code:: bash
	  
	  mkdir <install_location>/emdag
          cd emdag
	  git clone https://github.com/pshriwise/emdag emdag_src
	  mkdir bld
	  cd bld
	  cmake ../emdag_src -DCMAKE_INSTALL_PREFIX=../
	  export LD_LIBRARY_PATH=<install_location>/emdag/lib:$LD_LIBRARY_PATH

Finally, complete the DAGMC installation.
