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

	const float PI = 3.14159;
	return vec3(alt, lon, lat);
}

vec3 computeEquirectangular(vec3 world_pos){
	vec3 all = ecef2all(world_pos);
	float alt = all.x;
	float lon = all.y;
	float lat = all.z;
	return vec3(alt, lon * EARTH_RADIUS_MAX, lat * EARTH_RADIUS_MAX);
}

vec3 projectPointOnEllipsoid(vec3 world_pos){
	vec3 sensor_origin_wc = czm_model[3].xyz;

	//http://www.ambrsoft.com/TrigoCalc/Sphere/SpherLineIntersection_.htm
	vec3 slope = world_pos - sensor_origin_wc;
	float a = dot(slope, slope);
	float b = -2. * dot(slope, -sensor_origin_wc);
	float c = dot(sensor_origin_wc, sensor_origin_wc) - EARTH_RADIUS_MAX * EARTH_RADIUS_MAX;

	float discriminant = b * b - 4. * a * c;
	float t1 = (-b + sqrt(discriminant)) / (2. * a);
	float t2 = (-b - sqrt(discriminant)) / (2. * a);
	float t = min(t1, t2);	//slightly elongate to account for error

	vec3 projected_point = sensor_origin_wc + slope * t;
	//if a is 0 then it is the same point as sensor origin
	bool is_intersecting = discriminant > 0.;
	bool is_not_sensor_origin = a > .001;

	return is_intersecting && (t > 0.) ? projected_point : world_pos;
}

vec3 computeScenePosition(vec3 world_pos){
	if(czm_sceneMode == czm_sceneMode3D){
		return world_pos;
	}

	vec3 projected_pos = computeEquirectangular(world_pos);

	return mix(projected_pos, world_pos, czm_morphTime);
}

void main()
{
	vec4 world_pos = czm_model * position;
	vec3 projected_world_pos = projectPointOnEllipsoid(world_pos.xyz);
  vec3 scene_wc = computeScenePosition(projected_world_pos);


  gl_Position = czm_viewProjection * vec4(scene_wc, 1.);

  //Use regular transformation to trick the sensor into thinking its in 3d space
  //The fragment calculation turns out to be exactly the same with discarding the
  //pixels intersecting the Wgs84 globe
  v_positionWC = projected_world_pos.xyz;
  v_positionEC = (czm_view * vec4(projected_world_pos, 1.)).xyz;
  v_normalEC = czm_normal * normal;
}
