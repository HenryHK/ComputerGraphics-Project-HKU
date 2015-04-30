#Intro

features: 

	1) basic: shadow, mirror reflection <br\>
	2) optional: <br\>
		1. flattening scene graph: <br\>
			void flattenScene(SceneDagNode* node, Matrix4x4 parentMTW, <br\>
				Matrix4x4 parentWTM) <br\>
		2. Refraction:<br\>
			using Snell's law for objects which have two field namely "transmissive" <br\>
			and "refrac_idx" representing whether it is transmissive and index of refraction.<br\>
			See the definition of Material struct for reference. For a ray inside a object,<br\>
			using -normal instead of normal(see void shade(Ray3D& ray) in lightsource.cpp <br\>
			for reference).<br\>
		3. Anti-aliasing:<br\>
			Using super-sampling to do anti-aliasing work.<br\>
		4. Texture mapping:<br\>
			Give material a texture field. Need to update universal coordinates in scene_object <br\>
			in mapping process.<br\>
		5. Area light source:<br\>
			Add a function called "AddHeadLightSourceLight()" which give a group of point light <br\>
			simulating a head light. But the soft shadows feature is more obvious with random ray <br\>
			which, meanwhile, needs a high level of anti-aliasing to avoid the dot-like edge of <br\>
			the shadow.<br\>
		6. Glossy reflection:<br\>
			Get a random reflection direction by generating cosine weighted random hemisphere samples<br\>
		7. Gamma Correction:<br\>
			Just for fun. Using 2.2 as default value.<br\>