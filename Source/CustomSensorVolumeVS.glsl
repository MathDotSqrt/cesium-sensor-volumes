attribute vec4 position;
attribute vec3 normal;

varying vec3 v_positionWC;
varying vec3 v_positionEC;
varying vec3 v_normalEC;

vec4 computePosition(vec4 position3D){
	
	vec4 p = czm_model * position3D;
	if(czm_sceneMode == czm_sceneMode3D){
		return p;
	}

	float radius = length(p.xyz);
	float lat = asin(p.z / radius) * 40075000. / ( 2. * 3.14159);
	float lon = atan(p.y, p.x) * 40075000. / (2. * 3.14159);
	return vec4(radius / 10., lon, lat, 1);
}

void main()
{

    vec4 world_pos = computePosition(position);


    gl_Position = czm_viewProjection * world_pos;
    v_positionWC = (czm_model * position).xyz;
    v_positionEC = (czm_modelView * position).xyz;
    v_normalEC = czm_normal * normal;
}
