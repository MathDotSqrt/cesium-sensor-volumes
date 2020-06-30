attribute vec4 position;
attribute vec3 normal;

varying vec3 v_positionWC;
varying vec3 v_positionEC;
varying vec3 v_normalEC;

const float EARTH_RADIUS_MAX = 6378137.;	//max radius of the earth
const float EARTH_RADIUS_MIN = 6356752.314; 	//min radius of the earth

float geocentricRadius(float lat){
	const float a = EARTH_RADIUS_MAX;
	const float a2 = a * a;
	const float b = EARTH_RADIUS_MIN;
	const float b2 = b * b;

	float sin_lat = sin(lat);
	float cos_lat = cos(lat);
	
	float a2_cos_lat = a2 * cos_lat;
	float a2_cos_lat2 = a2_cos_lat * a2_cos_lat;

	float b2_sin_lat = b2 * sin_lat;
	float b2_sin_lat2 = b2_sin_lat * b2_sin_lat;

	float a_cos_lat = a * cos_lat;
	float a_cos_lat2 = a_cos_lat * a_cos_lat;

	float b_sin_lat = b * sin_lat;
	float b_sin_lat2 = b_sin_lat * b_sin_lat;

	return sqrt((a2_cos_lat2 + b2_sin_lat2) / (a_cos_lat2 + b_sin_lat2));
}

vec3 cartesian2Spherical(vec3 cartesian){
	float radius = length(cartesian);
	float lat = asin(cartesian.z / radius);
	float lon = atan(cartesian.y, cartesian.x);

	float earth_radius = geocentricRadius(lat);
	float alt = radius - earth_radius;

	return vec3(alt, lon, lat);
	
}

//https://www.mathworks.com/matlabcentral/fileexchange/7941-convert-cartesian-ecef-coordinates-to-lat-lon-alt
vec3 ecef2all(vec3 ecef){
	const float a = EARTH_RADIUS_MAX;
	const float a2 = a * a;

	const float e = 8.1819190842622E-2;
	const float e2 = e * e;

	const float b = sqrt(a2*(1.-e2));
	const float b2 = b * b;
	const float ep = sqrt((a2 - b2) / b2);
	const float ep2 = ep * ep;

	float p = sqrt(ecef.x * ecef.x + ecef.y * ecef.y);
	float th = atan(a * ecef.z, b * p);

	float sin_th = sin(th);
	float sin_th3 = sin_th * sin_th * sin_th;
	float cos_th = cos(th);
	float cos_th3 = cos_th * cos_th * cos_th;

	float lon = atan(ecef.y, ecef.x);
	float lat = atan(ecef.z + ep2 * b * sin_th3, p - e2 * a * cos_th3);

	float sin_lat = sin(lat);
	float sin_lat2 = sin_lat * sin_lat;
	float N = a / sqrt(1. - e2 * sin_lat2);

	float alt = p / cos(lat) - N;

	return vec3(alt, lon, lat);
}

vec4 computeEquirectangular(vec4 world_pos){
	vec3 all = ecef2all(world_pos.xyz);
	
	//float radius = length(world_pos.xyzw);
	//float lat = asin(world_pos.z / radius) * EARTH_RADIUS ;
	//float lon = atan(world_pos.y, world_pos.x) * EARTH_RADIUS;

	return vec4(all.x, all.y * EARTH_RADIUS_MAX, all.z * EARTH_RADIUS_MAX, 1);

	//world_pos.x *= 3.;
	//world_pos.y *= 3.;
	//world_pos.z *= 3.;
	

	//return world_pos;
}

vec4 computePosition(vec4 position3D){
	
	vec4 p = czm_model * position3D;
	if(czm_sceneMode == czm_sceneMode3D){
		return p;
	}


	vec4 projected_pos = computeEquirectangular(p);

	return mix(projected_pos, p, czm_morphTime);
	//return p;
}

void main()
{

    vec4 world_pos = computePosition(position);


    gl_Position = czm_viewProjection * world_pos;
    
    //Use regular transformation to trick the sensor into thinking its in 3d space
    //The fragment calculation turns out to be exactly the same with discarding the 
    //pixels intersecting the Wgs84 globe
    v_positionWC = (czm_model * position).xyz;
    v_positionEC = (czm_modelView * position).xyz;
    v_normalEC = czm_normal * normal;
}
