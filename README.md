features: 
	1) basic: shadow, mirror reflection
	2) optional: 
		1. flattening scene graph: 
			void flattenScene(SceneDagNode* node, Matrix4x4 parentMTW, 
				Matrix4x4 parentWTM) 
		2. Refraction:
			using Snell's law for objects which have two field namely "transmissive" 
			and "refrac_idx" representing whether it is transmissive and index of refraction.
			See the definition of Material struct for reference. For a ray inside a object,
			using -normal instead of normal(see void shade(Ray3D& ray) in lightsource.cpp 
			for reference).
		3. Anti-aliasing:
			Using super-sampling to do anti-aliasing work.
		4. Texture mapping:
			Give material a texture field. Need to update universal coordinates in scene_object 
			in mapping process.
		5. Area light source:
			Add a function called "AddHeadLightSourceLight()" which give a group of point light 
			simulating a head light. But the soft shadows feature is more obvious with random ray 
			which, meanwhile, needs a high level of anti-aliasing to avoid the dot-like edge of 
			the shadow.
		6. Glossy reflection:
			Get a random reflection direction by generating cosine weighted random hemisphere samples
		7. Gamma Correction:
			Just for fun. Using 2.2 as default value.