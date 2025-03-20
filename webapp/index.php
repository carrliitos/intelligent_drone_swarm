<?php

    if ($_SERVER["REQUEST_METHOD"] == "POST")
     {
        //using absolute path, meaning $scriptPath needs to be modifed to fit our personal paths
        $venvPython = "/Users/griffinpolly/Downloads/esp32_drone_swarm/venv/bin/python";
        $scriptPath = "/Users/griffinpolly/Downloads/esp32_drone_swarm/src/main.py";

        if (isset($_POST["start_script"])) {
                $cmd = escapeshellarg($venvPython) . " " . escapeshellarg($scriptPath) . " 2>&1";
                $output = shell_exec($cmd);
                echo "Output:\n$output";
                exit();

        }
        //stop function is not currently working, the only thing it stops is the logs from being displayed. Will be working on this further next.
        if (isset($_POST["stop_script"])) {
                shell_exec("pkill -2 -f " . escapeshellarg($scriptPath)); //pkill -2 = cntrl + c
                header('Content-Type: text/plain');
                echo "Script stopped.";
                exit();
        
    }
}
?>