<?php

echo '<?xml version="1.0" encoding="UTF-8"?>';  

function notIE(){
	$msie='/msie\s(5\.[5-9]|[6-9]\.[0-9]*).*(win)/i';
	if( !isset($_SERVER['HTTP_USER_AGENT']) ||
        !preg_match($msie,$_SERVER['HTTP_USER_AGENT']) ||
        preg_match('/opera/i',$_SERVER['HTTP_USER_AGENT']))
        return true;
	return false;
}

?>

<!DOCTYPE html 
     PUBLIC "-//W3C//DTD XHTML 1.0 Transitional//EN"
     "DTD/xhtml1-transitional.dtd">
    
<html xmlns="http://www.w3.org/1999/xhtml" xml:lang="en" lang="en">
<head>
    <meta name="keywords" content="simon physic dynamic engine university koblez marionett" />    
    <meta name="description" content="A dynamics simulation library for realtime environments." />    
    <meta name="robots" content="index, follow" />

    <link rel="stylesheet" type="text/css" href="base.css"/>
    <title>S I M O N, simulation of dynamics</title>

    <link rel="icon" href="favicon.ico" type="image/ico" /> 
    <link rel="SHORTCUT ICON" href="favicon.ico" />

</head>

<body><div id="body">

            <div id="header">
	<!-- IE Transparency hack. Look into the css file for mor -->
                	<div id="logo" style="<?php if (notIE()) {echo "background-image:url('images/logo.png')";} ?>">
	<img border="0" src="images/spacer" alt="Simon logo"/>
                	</div>
	            <?php include "navigation.php"; ?>
            </div>

            <div id="content">