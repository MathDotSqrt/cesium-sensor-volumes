attribute vec4 position;
attribute vec3 normal;

varying vec3 v_positionWC;
varying vec3 v_positionEC;
varying vec3 v_normalEC;

const float EARTH_CIRCUMFERENCE = 40075000.;
const float RADIAN_TO_METERS = EARTH_CIRCUMFERENCE / czm_twoPi;
const float EARTH_RADIUS = 6371000.;

vec4 computeEquirectangular(vec4 world_pos){
	float radius = length(world_pos.xyz);
	float lat = asin(world_pos.z / radius) * RADIAN_TO_METERS;
	float lon = atan(world_pos.y, world_pos.x) * RADIAN_TO_METERS;

	return vec4(radius - EARTH_RADIUS, lon, lat, 1);
}

vec4 computePosition(vec4 position3D){
	
	vec4 p = czm_model * position3D;
	if(czm_sceneMode == czm_sceneMode3D){
		return p;
	}


	vec4 projected_pos = computeEquirectangular(p);

	return mix(projected_pos, p, czm_morphTime);
}

void main()
{

    vec4 world_pos = computePosition(position);


    gl_Position = czm_viewProjection * world_pos;
    v_positionWC = (czm_model * position).xyz;
    v_positionEC = (czm_modelView * position).xyz;
    v_normalEC = czm_normal * normal;
}
