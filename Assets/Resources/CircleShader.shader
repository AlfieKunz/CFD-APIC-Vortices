//Fragment shader for displaying the particles to the screen (generalised as blue circles of fixed radii).
Shader "Unlit/CircleShader"
{
    SubShader
    {
        Tags { "Queue"="Transparent" "RenderType"="Transparent" }
        Blend SrcAlpha OneMinusSrcAlpha
        Cull Off

        Pass
        {
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma target 4.5 //For use in buffers.
            #pragma multi_compile_instancing

            #include "UnityCG.cginc"

            
            #define ColorSlow float4(0.13, 0.70, 0.67, 1.0)
            #define ColorMedium float4(1.0, 0.5, 0.0, 1.0)
            #define ColorFast float4(1.0, 0.0, 0.0, 1.0)
            
            //Data fetched by the Compute Buffer - particle data.
            float RenderSize;
            float MaxVelocity;
            struct Particle {
                float2 pos;
                float2 velocity;
            };
            StructuredBuffer<Particle> Particles;
            
            //Vertex data. uv symbolises circular mesh.
            struct v2f {
                float2 uv : TEXCOORD0;
                float4 vertex : SV_POSITION;
                float4 color : COLOR0;
            }; 

            sampler2D _MainTex;
            float4 _MainTex_ST;

            //Vertex shader (positions of all the vertices specified by the mesh).
            v2f vert (appdata_full v, uint instanceID : SV_InstanceID) {
                v2f o;
                Particle data = Particles[instanceID];
                float3 worldPos = v.vertex.xyz * RenderSize + float3(data.pos, 0); //Moves the mesh to its desired location on the screen.
                o.vertex = UnityObjectToClipPos(float4(worldPos, 1.0f)); //Converts to clip space, so that the GPU can render (discrete).
                o.uv = v.texcoord;

                float RelativeVelocity = saturate(length(data.velocity) / MaxVelocity);
                if (RelativeVelocity <= 0.67) {
                    o.color = lerp(ColorSlow, ColorMedium, RelativeVelocity * 3/2);
                } else {
                    o.color = lerp(ColorMedium, ColorFast, (RelativeVelocity - 0.67) * 3);
                }
                return o;
            }

            //Fragment shader (colour of all the pixels on the mesh).
            fixed4 frag (v2f i) : SV_Target {
                float dist = distance(i.uv, float2(0.5, 0.5)); //Procedural drawing.
                float alpha = 1.0 - smoothstep(0.48, 0.5, dist); //Creates circular shape from the square mesh, using the distance function.
                clip(alpha - 0.01); //Discards any pixels that are almost fully transparent.
                return fixed4(i.color.rgb, alpha);
            }
            ENDCG
        }
    }
}
