<?php include "header.php"; ?>

<div id="left">
	<h2>Files</h2>
	
	<p>Grab our first developement snapshot package 
	<a href="http://download.berlios.de/opensimon/simon-0.18.tar.gz">
	   simon-0.18.tar.gz</a> from the berliOS site. <br/><br/>

	This tarball is only for test purposes. We have no stable
	releases at the moment.</p>
</div>

<div id="right">
	<h2>How to install?</h2>
<h3>Dependencies</h3>
	<p>We make heavy use of the <a href="http://www.boost.org/">boost library</a>. You will also need OpenGL and the Autotools to compile our library.<br/>
Flex and Bison will be neccessary only for our internal test application.
</p>

<h3>Platforms</h3>
<p>Any Unix should be able to compile Simon. We also have Visual Studio project files for those who are not able to get something with an "x" in the name.</p>
</div>

<div id="single">
	  <h2>Getting the latest and greatest</h2>

<p>
	We use a <a href="http://svn.berlios.de/viewcvs/opensimon">Subversion
	repository</a> at berliOS for developement.<br/><br/>

	To get the developemend HEAD of simon just type <br/>
	
	<i>svn checkout svn://svn.berlios.de/opensimon/trunk</i><br/>
	If you don't have Subversion installed get it at 
	<a href="http://subversion.tigris.org/">http://subversion.tigris.org/</a><br/><br/>
	
	For compiling the source look into the INSTALL file.
</p>

</div>

 <?php include "footer.php"; ?>
