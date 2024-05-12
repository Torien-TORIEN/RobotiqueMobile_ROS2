# checking the input of the script

# -----------------------------------
# GLOBAL VARIABLES
# -----------------------------------


$image_tag = "ros2:user"
$image_file = ".\ros2_user.tar.gz"
$container_name = "ros2"

$docker_compose_file = "docker-compose-novnc.yml"
$network_name = "ros_netw"


# -----------------------------------
# SETTING VARIABLES
# -----------------------------------

$docker_desktop_path = "C:\Program Files\Docker\Docker\Docker Desktop.exe"

# -----------------------------------
# DOCKER DESKTOP
# -----------------------------------
Write-Host "Checking Docker Desktop"
if(!(Get-Process "Docker Desktop" -ErrorAction SilentlyContinue)){
	Write-Host "Starting Docker Desktop..."
	# start docker desktop
	& "$docker_desktop_path"
}else{
	Write-Host "Docker Desktop is already running..."
}

do{
	$dockerStatus = docker info 2>&1 | Select-String "Server Version"
	if(!$dockerStatus){
		Write-Host "Waiting for Docker engine to start (can take a while)..."
		Start-Sleep -Seconds 1
	}
}until($dockerStatus)
Write-Host "Docker Engine is running!"

# -----------------------------------
# REMOVING CONTAINERS
# -----------------------------------
$containers = docker ps -aq
if($containers){
	Write-Host "Stopping and removing all the current containers..."
	docker stop $containers | docker rm $containers
}else{
	Write-Host "No containers to remove"
}

# -----------------------------------
# CHECKING IMAGE
# -----------------------------------
Write-Host "Checking if the <$image_tag> image exists"
$image = docker images -q $image_tag
if($image){
	Write-Host "The image <$image_tag> already exists"
}else{
	Write-Host "The image <$image_tag> does not exists, it needs to be loaded..."
	Write-Host "Trying to load <$image_file>, can take a while..."
	docker load -i $image_file
}

# -----------------------------------
# DEALING WITH DOCKER NETWORK
# -----------------------------------

Write-Host "Checking the docker <$network_name> network..."
if(!(docker network inspect $network_name -f '{{.Name}}' 2>$null) ){
	Write-Host "Creating the <$network_name> docker network"
	docker network create --driver bridge $network_name
}
else{
	Write-Host "Docker network <$network_name> already exists"
}

# -----------------------------------
# DEALING WITH DOCKER COMPOSE
# -----------------------------------

Write-Host "Starting docker compose"
docker compose -f $docker_compose_file up -d


# -----------------------------------
# DEALING WITH EXECUTION
# -----------------------------------

Write-Host "Waiting 5s for the containers novnc to start..."
Start-Sleep -Seconds 5
& 'C:\Program Files\BraveSoftware\Brave-Browser\Application\brave.exe' "http://127.0.0.1:8080/vnc_auto.html"
#& 'C:\Program Files\Mozilla Firefox\firefox.exe' "http://127.0.0.1:8080/vnc_auto.html"
