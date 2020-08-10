attribute vec4 position;
attribute vec3 normal;

varying vec3 v_positionWC;
varying vec3 v_positionEC;
varying vec3 v_normalEC;

const float EARTH_RADIUS_MAX = 6378137.;			//max radius of the earth
const float EARTH_RADIUS_MIN = 6356752.314; 	//min radius of the earth

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

float wrapLon(float lon){
	vec3 origin = czm_model[3].xyz / czm_model[3].w;

	//longitude of the center of the sensor
	float origin_lon = atan(origin.y, origin.x);

	float lon_right = mod(lon, czm_twoPi);
	float lon_left = lon_right - czm_twoPi;

	if(origin_lon > 1.){
		return lon_right;
	}
	if(origin_lon < -1.){
		return lon_left;
	}

	return lon;
}

vec3 computeEquirectangular(vec3 world_pos){
	//converts earth centered earth fixed coordinates to
	//geodetic altitude, longitude, latitude
	vec3 all = ecef2all(world_pos);
	float alt = all.x;
	float lon = all.y;
	float lat = all.z;

	//longitude normally has range of -pi, +pi
	//but if sensor is at the edge of the 2d scene mapProjection
	//parts of the sensor will tear
	lon = wrapLon(lon);

	//actual equirectangular projection
	//TODO perform mercator projection as an option
	vec3 projection = vec3(alt, lon * EARTH_RADIUS_MAX, lat * EARTH_RADIUS_MAX);

	return projection;
}

vec3 computeScenePosition(vec3 world_pos){
	if(czm_sceneMode == czm_sceneMode3D){
		//scenemode: 3D position
		return world_pos;
	}

	//sceneMode: 2D, Columbus position
	vec3 projected_pos = computeEquirectangular(world_pos);

	//interpolate between them during scene morph (scene mode transition)
	return mix(projected_pos, world_pos, czm_morphTime);
}

vec3 projectPointOnEllipsoid(vec3 world_pos){
	vec3 sensor_origin_wc = czm_model[3].xyz / czm_model[3].w;

	//http://www.ambrsoft.com/TrigoCalc/Sphere/SpherLineIntersection_.htm
	vec3 slope = world_pos - sensor_origin_wc;
	float a = dot(slope, slope);
	float b = -2. * dot(slope, -sensor_origin_wc);
	float c = dot(sensor_origin_wc, sensor_origin_wc) - EARTH_RADIUS_MIN * EARTH_RADIUS_MIN;

	float discriminant = b * b - 4. * a * c;
	float sqrt_discriminant = sqrt(discriminant);
	float t1 = (-b + sqrt_discriminant) / (2. * a);
	float t2 = (-b - sqrt_discriminant) / (2. * a);
	float t = min(t1, t2);

	vec3 projected_point = sensor_origin_wc + slope * t;

	//bool is_not_sensor_origin = a > .001;
	bool is_intersecting = discriminant > 0.;
	return is_intersecting && (t > 0.) ? projected_point : world_pos;
}

void main()
{
	vec4 world_pos = czm_model * position;

	//if position intersects the earth, project position onto surface
	//of the ellipsoid.
	//Slight performance boost because fragment shader will have to cull
	//fewer pixels.
	//This also aids with the accuracy of the 2d projection
	vec3 projected_world_pos = projectPointOnEllipsoid(world_pos.xyz);

	//computes position with respect to cesium's scene mode 3d, 2d,
	//columbus, and morph
  vec3 scene_wc = computeScenePosition(projected_world_pos);

  gl_Position = czm_viewProjection * vec4(scene_wc, 1.);

  //Use regular transformation to trick the sensor into thinking its in 3d scene mode
  //The fragment calculation for discarding intersecting pixels
  //with the Wgs84 globe is the same in all scene modes
  v_positionWC = projected_world_pos;
  v_positionEC = (czm_view * vec4(projected_world_pos, 1.)).xyz;
  v_normalEC = czm_normal * normal;
}
