<!DOCTYPE HTML> 
<html>
<head>
<style>
.error {color: #FF0000;}
</style>
</head>
<body> 

<?php

function checkRunning() {
	exec("ps aux | grep -i 'mw-mavlink' | grep -v grep", $pids);
	if(empty($pids)) {
		return 0;
	} else {
		return 1;
	}
}

function myip() {
	$cmd="/sbin/ifconfig $1 | grep \"inet addr\" | awk -F: '{print $2}' | awk '{print $1}'";
	$out = exec($cmd,$ret);
	for ($i=0;$i<count($ret);$i++) {
        if (substr_compare($ret[$i],'127.',0,4)!=0)
                $host = $ret[$i];
    }

    return $host;
}

function saveConfig($gsip,$gsport,$lport) {
	$cmd="echo '-t ".$gsip." -p ".$gsport." -l ".$lport."' > /etc/mwmav.config";
	$out = exec($cmd,$ret);
	$cmd="sync;sync;";
	$out = exec($cmd,$ret);
}

function readConfig() {
	$cmd="cat /etc/mwmav.config";
	$out = exec($cmd,$ret);
    return $ret[0];
}

// define variables and set to empty values
$gsip = $gsport = $lport = "";
$error = 0;
$save = 0;

if ($_SERVER["REQUEST_METHOD"] == "POST") {
	$save = 1;
   if (empty($_POST["gsip"])) $error = 1;
   else $gsip = $_POST["gsip"];
   
   if (empty($_POST["gsport"])) $error = 1;
   else $gsport = $_POST["gsport"];

   if (empty($_POST["lport"])) $error = 1;
   else $lport = $_POST["lport"];
}
?>
<a href="index.php">Refresh</a>
<h2>MultiWii MavLink config</h2>
<?php
if (checkRunning()) echo "mw-mavlink seems to be running<br/>";
else echo "mw-mavlink not running?<br/>";

echo "My IP: ".myip();
echo "<br/><br/>";
?>
<form method="post" action="<?php echo htmlspecialchars($_SERVER["PHP_SELF"]);?>"> 
   GroundStation IP: <input type="text" name="gsip" value="<?php echo $gsip;?>">
   <br><br>
   GroundStation Port <input type="text" name="gsport" value="<?php echo $gsport;?>">
   def: 14550
   <br><br>
   Local port: <input type="text" name="lport" value="<?php echo $lport;?>">
   def: 14551
   <br><br>
   <input type="submit" name="submit" value="Submit"> 
</form>


<?php
if ((!$error) && ($save)) {
	echo "<h2>Saved!</h2>";
	saveConfig($gsip,$gsport,$lport);
}

echo "<h2>Current config:</h2>";
echo readConfig();
echo "<br/><br/>";
?>

</body>
</html>
