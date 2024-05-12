docker network create --driver bridge ros_netw || true
docker compose -f docker-compose-novnc.yml up -d
