using Unity.Mathematics;
using UnityEngine;
using System;
using System.Runtime.InteropServices;
using System.Threading.Tasks;
using System.IO;

//Class holding the Fluids Simulation, as well as the data needed to render the fluid to the screen, via a fragment shader.
[Serializable]
public partial class FluidSim : MonoBehaviour {

    public GlobalSettings Settings;


    //Info for the renderer.
    public float ParticleRenderSize;
    public float MaxVelocity = 50f;
    public ComputeBuffer ParticleBuffer;
    public Material ParticleMaterial;
    public Mesh ParticleQuadMesh;
    public Bounds RenderBounds;

    private ParticleRender[] ParticleRenderCont;


    //General & User controls.
    public float SimulationTime;
    private float UserScrubNextStep = 0f; // Time stamp for allowing the user to scrub through the simulation (by holding the right arrow key).
    private float deltaTime; // Time between each simulation step.
    private bool HasBeenInitialised = false;

    private int[] SolidCellLookup; //Holds all the cell indices that are solid, so that they can be found easily.
    private int[] NonSolidCellsOrdered; // Stores the indices of all non-solid cells, in ascending order from distance to the simulation centre point.

    private string FilePath;    
    private StreamWriter DataWriter;

    Unity.Mathematics.Random RNG;



    void Start() {
        Settings.TimeStep = 1 / 60f;
        Init();
        DrawParticles();
    }
    public void Init() {
        //Initialises the grid & particles, along with core settings.
        Settings.isRunning = false;
        SimulationTime = 0f;
        Time.fixedDeltaTime = Settings.TimeStep;
        RNG = new Unity.Mathematics.Random((uint)DateTime.Now.Millisecond + 1);

        if (DataWriter != null) {
            DataWriter.Close();
            DataWriter.Dispose();
        }
        FilePath = Path.Combine(Application.dataPath, $"Simulation Logs/SimLog_{DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss")}.csv");
        DataWriter = new StreamWriter(FilePath);

        InitGrid();
        InitParticles();
        HasBeenInitialised = true;
    }


    public void InitGrid(bool CreateObjects = true) {
        //Initialises the dimensions of the grid via the Global Settings, then creates the Mask (solid vs liquid).
        int SolidCellCount = 0, NonSolidCellCount = 0;
        if (CreateObjects) {
            GridWidth = Settings.GridDimensions.x;
            GridHeight = Settings.GridDimensions.y;
            CellSize = CameraController.WorldHeight / GridHeight;
            GridSize = GridWidth * GridHeight;
            SimCentre = new float2(GridWidth, GridHeight) * CellSize / 2f;
            Debug.Log("Cellsize is: " + CellSize);

            GridMap = new Grid(GridSize, Settings.GridDimensions);
            SolidCellLookup = new int[GridSize];
            NonSolidCellsOrdered = new int[GridSize];
            PCG = new PreconditionedConjugateGradient(GridSize, Settings.ProjectionStepSize, Settings.PeriodicBCs, ref GridMap, GridWidth, GridHeight);
        }

        int SolidCellBorder = 1;
        for (int n = 0; n < GridMap.Length(); n++) {
            //Inits the grid position.
            if (CreateObjects) {
                (int i, int j) = Grid1DUnfoldIndex(n);
                GridMap.InitCell(i, j);
                if (i < SolidCellBorder || i >= GridWidth - SolidCellBorder || j < SolidCellBorder || j >= GridHeight - SolidCellBorder) {
                    GridMap.Type[n] = 0; //represents solids (boundary).
                    SolidCellLookup[SolidCellCount] = n;
                    SolidCellCount++;
                } else {
                    // The cell is not a solid cell - add this to a lookup table of indices, sorted by the heuristic chosen by the spawn type algorithm.
                    NonSolidCellsOrdered[NonSolidCellCount] = n;
                    NonSolidCellCount++;

                    Vector2 GridCentre = new(i + 0.5f, j + 0.5f);
                    switch (Settings.SpawnType) {
                        case GlobalSettings.ParticleSpawnType.CentreCircle:
                            // As we spawn the fluid in from the centre, in a circular pattern, we need to know the distance from each non solid cell's centre to the centre of the simulation.
                            Vector2 SimCentre = new(GridWidth / 2f, GridHeight / 2f);
                            GridMap.DistanceSortValues[n] = Vector2.Distance(GridCentre, SimCentre);
                            break;
                        case GlobalSettings.ParticleSpawnType.CentreSquare:
                            // Similar for the above case, but now we spawn everything as a square, and thus use the Manhatten norm.
                            Vector2 DistanceToCentreBottom = math.abs(GridCentre - new Vector2(GridWidth / 2f, 0.5f));
                            GridMap.DistanceSortValues[n] = math.max(2.5f * DistanceToCentreBottom.x, DistanceToCentreBottom.y);
                            break;
                        case GlobalSettings.ParticleSpawnType.CentreBottomRectangle:
                            // Spawn method that fills the bottom with fluid, leaving a slight delta that allows _some_ motion to happen at the start of the simulation.
                            int Delta = 3;
                            Vector2 DistanceToCentreBottomDelta = GridCentre - new Vector2(GridWidth / 2f, Delta + 0.5f);
                            if (DistanceToCentreBottomDelta.y < 0 || Delta > GridCentre.x || GridCentre.x > GridWidth - Delta) {
                                GridMap.DistanceSortValues[n] = float.MaxValue;
                            } else {
                                GridMap.DistanceSortValues[n] = math.abs(DistanceToCentreBottomDelta.x) + GridWidth * math.abs(DistanceToCentreBottomDelta.y);
                            }
                            break;
                        case GlobalSettings.ParticleSpawnType.Random:
                            GridMap.DistanceSortValues[n] = RNG.NextInt(0, GridSize);
                            break;
                        case GlobalSettings.ParticleSpawnType.LHSSquare:
                                 // Place in a 'square' pattern from the bottom-left of the screen, that grows outwards. This is much easier to do by
                                 // constructing a growing lower triangle, stopping when index j reaches index i. For the upper part of the triangle, we
                                 // just repeat the same but add a small buffer (0.5) to ensure that the two triangles grow together.
                            GridMap.DistanceSortValues[n] = i >= j ? i * GridWidth + j : j * GridWidth + i + 0.5f;
                            break;
                        case GlobalSettings.ParticleSpawnType.LRRectangle:
                            // Similar for the above, but we also place fluid on the RHS of the simulation, to get more vortex action.
                            int iScaled = i < (GridWidth / 2f) ? i : GridWidth - i;
                            GridMap.DistanceSortValues[n] = iScaled >= j ? iScaled * GridWidth + j : j * GridWidth + iScaled + 0.5f;
                            break;
                        case GlobalSettings.ParticleSpawnType.Wave:
                            // Spawns particles in a wave structure, following A + HeightDelta*cos(kx), both in units of grid cells.
                            int ModeCount = 3;
                            int HeightDelta = 5;
                            GridMap.DistanceSortValues[n] = j - HeightDelta * math.sin(ModeCount * i * math.PI/GridWidth);
                            break;
                    }
                }
            } else {
                //Just resets all the fluid cells to air cells, and clears velocity and weright components.
                GridMap.ResetCell(n);
            }
        }

        // Puts a flag on the neighbouring cells to indicate that it has a solid neighbour.
        if (CreateObjects) {
            //Truncates Cell Lookup arrays.
            Array.Resize<int>(ref SolidCellLookup, SolidCellCount);
            Array.Resize<int>(ref NonSolidCellsOrdered, NonSolidCellCount);
            if (!Settings.PeriodicBCs) {
                for (int n = 0; n < GridMap.Length(); n++) {
                    if (GridMap.Type[n] == 0) {
                        (int i, int j) = Grid1DUnfoldIndex(n);
                        if (i > 0) { GridMap.NonSolidNeighbourCount[n - 1] -= 1; }
                        if (i < GridWidth - 1) { GridMap.NonSolidNeighbourCount[n + 1] -= 1; }
                        if (j > 0) { GridMap.NonSolidNeighbourCount[n - GridWidth] -= 1; }
                        if (j < GridHeight - 1) { GridMap.NonSolidNeighbourCount[n + GridWidth] -= 1; }
                    }
                }
            }
            // Sorts non solic cells by distance to sim centre.
            Array.Sort(NonSolidCellsOrdered, (i, j) => GridMap.DistanceSortValues[i].CompareTo(GridMap.DistanceSortValues[j]));
        }
    }

    public void InitParticles() {
        Settings.isRunning = false;
        ParticleCont = new Particle[Settings.ParticleCount];
        ParticleRenderCont = new ParticleRender[Settings.ParticleCount];
        float CellHalf = CellSize / 2f, CellOfset = CellSize / 4f;
        int ParticleCellCount = 0;
        int IndexInGrid = 0;
        //Places 4 particles in each cell, ofset by 1/4 of the CellSize, s.t they are all centred. Placed in a 'circle' pattern stemming from the 
        //centre of the screen.
        for (int i = 0; i < Settings.ParticleCount; i++) {
            ParticleCont[i] = new Particle(); //Initialises the particle.
            if (ParticleCellCount == 4) {
                //Cell has been filled - move onto the next cell.
                IndexInGrid++;
                if (IndexInGrid == NonSolidCellsOrdered.Length) {
                    //We are about to break the bounds of the simulation - break early, and truncate the particle array.
                    Array.Resize<Particle>(ref ParticleCont, i);
                    Array.Resize<ParticleRender>(ref ParticleRenderCont, i);
                    break;
                }
                ParticleCellCount = 0;
            }
            //Puts the particle in the right place in its cell (order: BL, BR, TL, TR).
            float2 ParticlePos = new float2(ParticleCellCount % 2, ParticleCellCount / 2);
            int2 CellIndex = GridMap.Index2D[NonSolidCellsOrdered[IndexInGrid]];
            float2 GridStart = new(CellIndex.x * CellSize, CellIndex.y * CellSize);
            ParticleCont[i].pos = CellOfset + (GridStart + (CellHalf * ParticlePos));

            //Adds some tiny 'jiggle' to the particles, to help the render not have any 'gaps' in them.
            float JiggleDelta = CellSize / 6f;
            ParticleCont[i].pos += RNG.NextFloat2(-JiggleDelta, JiggleDelta);

            if (Settings.InitialAngularMomentum != 0f) {
                float2 r = ParticleCont[i].pos - SimCentre;
                ParticleCont[i].velocity = new float2(-r.y, r.x) * Settings.InitialAngularMomentum;
                if (Settings.TransferMethod == GlobalSettings.TransferMethodType.AffinePIC) {
                    ParticleCont[i].cOperator_x = new float2(0, -Settings.InitialAngularMomentum);
                    ParticleCont[i].cOperator_y = new float2(Settings.InitialAngularMomentum, 0);
                }
            }
            if (Settings.InitialTGVVelocity != 0f) {
                float2 Mode = new(1f, 1f);
                float2 SinMultiplier = 2 * math.PI * new float2(Mode.x / (GridWidth - 2f), Mode.y / (GridHeight - 2f));
                float2 ScaledPos = ParticleCont[i].pos / CellSize - new float2(1f, 1f);

                ParticleCont[i].velocity.x = Settings.InitialTGVVelocity * math.sin(SinMultiplier.x * ScaledPos.x) * math.cos(SinMultiplier.y * ScaledPos.y);
                ParticleCont[i].velocity.y = -Settings.InitialTGVVelocity * math.cos(SinMultiplier.x * ScaledPos.x) * math.sin(SinMultiplier.y * ScaledPos.y);
                if (Settings.TransferMethod == GlobalSettings.TransferMethodType.AffinePIC) {
                    ParticleCont[i].cOperator_x = Settings.InitialTGVVelocity * new float2(
                        SinMultiplier.x * math.cos(SinMultiplier.x * ScaledPos.x) * math.cos(SinMultiplier.y * ScaledPos.y),   // du/dx
                        -SinMultiplier.y * math.sin(SinMultiplier.x * ScaledPos.x) * math.sin(SinMultiplier.y * ScaledPos.y));  // du/dy
                    ParticleCont[i].cOperator_y = Settings.InitialTGVVelocity * new float2(
                        SinMultiplier.x * math.sin(SinMultiplier.x * ScaledPos.x) * math.sin(SinMultiplier.y * ScaledPos.y),   // dv/dx
                        -SinMultiplier.y * math.cos(SinMultiplier.x * ScaledPos.x) * math.cos(SinMultiplier.y * ScaledPos.y));  // dv/dy
                }
            }

            ParticleCont[i].SetGridPosVars(CellSize);
            ParticleCellCount++;

            //Copies to particle render structure.
            ParticleRenderCont[i].Init(ref ParticleCont[i]);
        }


        PCG.Clear(Settings.ProjectionStepSize, Settings.PeriodicBCs);
        float StiffnessBuffer = Settings.Stiffness;
        Settings.Stiffness = 1;
        InterpolateParticlesToGrid();
        Settings.Stiffness = StiffnessBuffer;
        float TotalDensity = 0;
        for (int n = 0; n < GridMap.Length(); n++) {
            if (GridMap.Type[n] == 1) TotalDensity += GridMap.Density[n];
        }
        float AverageDensity = TotalDensity / PCG.GetSize();
        PCG.SetTargetDensity(AverageDensity);


        //Initialises the Rendering System:
        ParticleRenderSize = CellSize / 3f;
        //Creates a square mesh (that will become a circle via the uv's). Note the backwards-culling clockwise orientation.
        ParticleQuadMesh = new Mesh();
        ParticleQuadMesh.vertices = new Vector3[4] {
            new Vector3(-0.5f, 0.5f, 0),
            new Vector3(0.5f, 0.5f, 0),
            new Vector3(0.5f, -0.5f, 0),
            new Vector3(-0.5f, -0.5f, 0)
        };
        ParticleQuadMesh.triangles = new int[6] {
            0, 1, 2,  2, 3, 0 //2 triangles needed.
        };
        ParticleQuadMesh.uv = new Vector2[4] { //States coordinates of each vertex.
            new Vector2(0, 1),
            new Vector2(1, 1),
            new Vector2(1, 0),
            new Vector2(0, 0)
        };

        //Creates and sets the Particle Buffer, filling the compute shader with all the particle info.
        ParticleBuffer = new ComputeBuffer(ParticleRenderCont.Length, Marshal.SizeOf<ParticleRender>());
        ParticleBuffer.SetData(ParticleRenderCont);
        ParticleMaterial.SetBuffer("Particles", ParticleBuffer);
        ParticleMaterial.SetFloat("RenderSize", ParticleRenderSize);

        //We need to specify the bounds of the render material - set this via the camera's bounds.
        RenderBounds = new Bounds(
            new Vector3(CameraController.WorldWidth / 2f, CameraController.WorldHeight / 2f, 0),
            new Vector3(CameraController.WorldWidth, CameraController.WorldHeight, 10f)
        );
    }

    public void DrawParticles() {
        if (Settings.RenderParticles) {
            //I'll assume for the time being that all particles have the same size and colour (to make the rendering easy) :).
            //Tells the GPU to Procedurally draw the mesh (ie: same template for each particle).
            if (ParticleQuadMesh == null || ParticleMaterial == null || ParticleCont == null) {
                Debug.Log("Warning: Some Rendering Asset is null:");
                if (ParticleQuadMesh == null) { Debug.Log("Particle Quad Mesh"); }
                if (ParticleMaterial == null) { Debug.Log("Particle Material"); }
                if (ParticleCont == null) { Debug.Log("Particle Container"); }
                //Init();
            }
            ParticleMaterial.SetFloat("MaxVelocity", MaxVelocity);
            if (GPUReadyFlag) {
                //Prepares GPU for next frame, by copying all particle data to the (smaller) render structure.
                Parallel.For(0, ParticleCont.Length, i => { ParticleRenderCont[i].Init(ref ParticleCont[i]); });
                ParticleBuffer.SetData(ParticleRenderCont);
                GPUReadyFlag = false;
            }
            Graphics.DrawMeshInstancedProcedural(ParticleQuadMesh, 0, ParticleMaterial, RenderBounds, ParticleCont.Length);
        }
    }




    public void Run() { Settings.isRunning = true; }

    void FixedUpdate() {
        //Runs the simulation, given the pre-defined TimeStep.
        //if (Settings.isRunning) { SimulationTimer.Restart(); }
        if (Settings.isRunning) { RunSimlationStep(); }
        //if (Settings.isRunning) { SimulationTimer.Stop(); Debug.Log("Simulation complete in: " + SimulationTimer.Elapsed.TotalMilliseconds + "ms."); }
    }

    void Update() {
        //Handle user inputs.
        if (Input.GetKeyDown(KeyCode.Space)) {
            //SPACE: Play/Pause the simulation.
            Settings.isRunning = !Settings.isRunning;
        }

        if (Input.GetKeyDown(KeyCode.RightArrow) && !Settings.isRunning) {
            //RIGHT ARROW: Runs the simulation for one frame, then schedules a bulk of frames, if the user is holding down the key.
            RunSimlationStep();
            UserScrubNextStep = Time.time + 0.25f;
        } else if (Input.GetKey(KeyCode.RightArrow) && !Settings.isRunning) {
            //User is holding down the key - run 'slowed down' version of the simulation.
            if (Time.time >= UserScrubNextStep) {
                RunSimlationStep();
                UserScrubNextStep = Time.time + Settings.TimeStep * 2;
            }

        } else if (Input.GetKeyDown(KeyCode.LeftArrow) && !Settings.isRunning) {
            //LEFT ARROW: Runs the simulation for one frame, with a negative time (runs backwards). Similar for RightArrow, we can run a bulk of frames.
            Settings.TimeStep *= -1;
            RunSimlationStep();
            Settings.TimeStep *= -1;
            UserScrubNextStep = Time.time + 0.25f;
        } else if (Input.GetKey(KeyCode.LeftArrow) && !Settings.isRunning) {
            //User is holding down the key - run a backwards 'slowed down' version of the simulation.
            if (Time.time >= UserScrubNextStep) {
                Settings.TimeStep *= -1;
                RunSimlationStep();
                Settings.TimeStep *= -1;
                UserScrubNextStep = Time.time + Settings.TimeStep;
            }
        } else if (Input.GetKey(KeyCode.R)) {
            //R: Reset the simulation.
            Start();
        }
        DrawParticles(); //Renders the full simulation to the screen.
    }

    void OnDestroy() {
        //Releases the Particle Buffers to prevent memory leaks.
        if (ParticleBuffer != null) {
            ParticleBuffer.Release();
            ParticleBuffer = null;
        }
        if (ParticleQuadMesh != null) {
            DestroyImmediate(ParticleQuadMesh);
            ParticleQuadMesh = null;
        }
        if (DataWriter != null) {
            DataWriter.Close();
            DataWriter.Dispose();
            DataWriter = null;
        }
    }

    private bool OnlyDrawFluidGizmos = true;
    void OnDrawGizmos() {
        if (HasBeenInitialised) {
            //(0,0) in the camera points to the centre; (0,0) in the simulation points to the bottom-left - CamOfset translates beteween these.
            Vector3 CamOfset = new Vector3(CameraController.WorldWidth / 2f, CameraController.WorldHeight / 2f, 0);
            Gizmos.color = new Color(0.5f, 0.5f, 0.5f, 0.25f); //A semi-transparent grey.

            //Draws horizontal lines.
            for (int x = 0; x <= GridWidth; x++) {
                float xPos = x * CellSize;
                Vector3 start = new Vector3(xPos, 0, 0);
                Vector3 end = new Vector3(xPos, CameraController.WorldHeight, 0);
                Gizmos.DrawLine(start + CamOfset, end + CamOfset);
            }
            //Draws vertical lines.
            for (int y = 0; y <= GridHeight; y++) {
                float yPos = y * CellSize;
                Vector3 start = new Vector3(0, yPos, 0);
                Vector3 end = new Vector3(CameraController.WorldWidth, yPos, 0);
                Gizmos.DrawLine(start + CamOfset, end + CamOfset);
            }

            //Fills in the solid cells (denoted by Mask = 0).
            for (int n = 0; n < GridMap.Length(); n++) {
                if (GridMap.Type[n] == 0) {
                    //Draws the cube, by finding the centre & radius.
                    (int x, int y) = Grid1DUnfoldIndex(n);
                    Vector3 Centre = new Vector3((x + 0.5f) * CellSize, (y + 0.5f) * CellSize, 0);
                    Vector3 Size = new Vector3(CellSize, CellSize, 0);
                    Gizmos.DrawCube(Centre + CamOfset, Size);
                    }
            }

            //Draws grid velocities to the screen.
            float DotSize = CellSize * (Settings.RenderParticles ? 0.1f : 0.2f);
            float GridSpeed;
            Vector3 DotPos;
            Color ColorSlow = new Color(0.13f, 0.70f, 0.67f, 1.0f);
            Color ColorMedium = new Color(1.0f, 0.5f, 0.0f, 1.0f);
            Color ColorFast = new Color(1.0f, 0.0f, 0.0f, 1.0f);
            for (int i = 0; i < GridMap.Length(); i++) {
                (int x, int y) = Grid1DUnfoldIndex(i);
                if (GridMap.Type[i] == 1 || !OnlyDrawFluidGizmos) {
                    for (int n = 1; n <= 2; n++) {
                        if (n == 1) {
                            GridSpeed = Math.Abs(GridMap.Velocity_GP[i].x);
                            DotPos = new Vector3((x + 1f) * CellSize, (y + 0.5f) * CellSize, 0);
                        } else {
                            GridSpeed = Mathf.Abs(GridMap.Velocity_GP[i].y);
                            DotPos = new Vector3((x + 0.5f) * CellSize, (y + 1f) * CellSize, 0);
                        }
                        if (GridSpeed < 1e-5) {
                            Gizmos.color = Color.grey;
                        } else {
                            float RelativeVelocity = Mathf.Clamp01(GridSpeed / MaxVelocity);
                            if (RelativeVelocity <= 0.67) {
                                Gizmos.color = Color.Lerp(ColorSlow, ColorMedium, RelativeVelocity * 3 / 2);
                            } else {
                                Gizmos.color = Color.Lerp(ColorMedium, ColorFast, (RelativeVelocity - 0.67f) * 3f);
                            }
                        }
                        Gizmos.DrawSphere(DotPos + CamOfset, DotSize);
                    }
                }  
            }

        }
    }

    //Ability to reset the simulation if the Global Settings have been changed.
    private void OnEnable() { GlobalSettings.SettingsChanged += HandleSettingsChanged; }
    private void OnDisable() { GlobalSettings.SettingsChanged -= HandleSettingsChanged; }
    private void HandleSettingsChanged(int SettingChangeCode) {
        switch (SettingChangeCode) {
            case 1:
                Time.fixedDeltaTime = Settings.TimeStep;
                break;
            case 2:
                InitParticles();
                break;
            case 3:
                Init();
                break;
        }
    }

    private bool AngularMomentumBoost = false;
    public void AddAngularMomentum() { // Adds angular momentum to the next frame.
        AngularMomentumBoost = true;
    }
    private bool SheerBoost = false;
    public void AddShear() { // Adds a shear force to the next frame.
        SheerBoost = true;
    }

}
