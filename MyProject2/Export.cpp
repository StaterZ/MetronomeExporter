#include "Export.h"
#include "Detour/DetourNavMesh.h"
#include "NavMesh/RecastNavMesh.h"
#include "NavigationSystem.h"
#include "NavigationSystem\Public\NavMesh\PImplRecastNavMesh.h"
#include "Components/LightComponent.h"
#include "Components/PointLightComponent.h"
#include "Components/SpotLightComponent.h"
#include "Components/DirectionalLightComponent.h"
#include "Kismet/GameplayStatics.h"
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

DEFINE_LOG_CATEGORY(LogExporter);

#pragma region Triangulation

#include <algorithm>
#include <vector>

#include "Camera/CameraComponent.h"
#include "EditorFramework/AssetImportData.h"

namespace delaunay
{
    constexpr double eps = 1e-4;

    template <typename T>
    struct Point
    {
        T x, y, z;

        Point() : 
            x{ 0 }, 
            y{ 0 }, 
            z{ 0 } 
        {}

        Point(T aX, T aY, T aZ) : 
            x{ aX }, 
            y{ aY }, 
            z{ aZ } 
        {}

        template<typename U>
        Point<U> Cast()
        {
            return {
                static_cast<U>(x), 
                static_cast<U>(y), 
                static_cast<U>(z)
            }
        }

        friend std::ostream& operator<<(std::ostream& os, const Point<T>& p)
        {
            os << "(" << p.x << "," << p.y << ")";
            return os;
        }

        bool operator==(const Point<T>& other) const
        {
            return x == other.x && y == other.y;
        }

        bool operator!=(const Point<T>& other) const { 
            return !(*this == other); 
        }
    };

    template <typename T>
    struct Edge
    {
        using Node = Point<T>;
        Node p0, p1;

        Edge(Node const& _p0, Node const& _p1) : p0{ _p0 }, p1{ _p1 } {}

        friend std::ostream& operator<<(std::ostream& os, const Edge& e)
        {
            os << "p0: [" << e.p0 << " ] p1: [" << e.p1 << "]";
            return os;
        }

        bool operator==(const Edge& other) const
        {
            return ((other.p0 == p0 && other.p1 == p1) ||
                (other.p0 == p1 && other.p1 == p0));
        }
    };

    template <typename T>
    struct Circle
    {
        T x, y, radius;
        Circle() = default;
    };

    template <typename T>
    struct Triangle
    {
        using Node = Point<T>;
        Node p0, p1, p2;
        Edge<T> e0, e1, e2;
        Circle<T> circle;

        Triangle(const Node& _p0, const Node& _p1, const Node& _p2)
            : p0{ _p0 },
            p1{ _p1 },
            p2{ _p2 },
            e0{ _p0, _p1 },
            e1{ _p1, _p2 },
            e2{ _p0, _p2 },
            circle{}
        {
            const auto ax = p1.x - p0.x;
            const auto ay = p1.y - p0.y;
            const auto bx = p2.x - p0.x;
            const auto by = p2.y - p0.y;

            const auto m = p1.x * p1.x - p0.x * p0.x + p1.y * p1.y - p0.y * p0.y;
            const auto u = p2.x * p2.x - p0.x * p0.x + p2.y * p2.y - p0.y * p0.y;
            const auto s = 1. / (2. * (ax * by - ay * bx));

            circle.x = ((p2.y - p0.y) * m + (p0.y - p1.y) * u) * s;
            circle.y = ((p0.x - p2.x) * m + (p1.x - p0.x) * u) * s;

            const auto dx = p0.x - circle.x;
            const auto dy = p0.y - circle.y;
            circle.radius = dx * dx + dy * dy;
        }
    };

    template <typename T>
    struct Delaunay
    {
        std::vector<Triangle<T>> triangles;
        std::vector<Edge<T>> edges;
    };

    template <
        typename T,
        typename = typename std::enable_if<std::is_floating_point<T>::value>::type>
        Delaunay<T> triangulate(const std::vector<Point<T>>& points)
    {
        using Node = Point<T>;
        if (points.size() < 3)
        {
            return Delaunay<T>{};
        }
        auto xmin = points[0].x;
        auto xmax = xmin;
        auto ymin = points[0].y;
        auto ymax = ymin;
        for (auto const& pt : points)
        {
            xmin = std::min(xmin, pt.x);
            xmax = std::max(xmax, pt.x);
            ymin = std::min(ymin, pt.y);
            ymax = std::max(ymax, pt.y);
        }

        const auto dx = xmax - xmin;
        const auto dy = ymax - ymin;
        const auto dmax = std::max(dx, dy);
        const auto midx = (xmin + xmax) / static_cast<T>(2.);
        const auto midy = (ymin + ymax) / static_cast<T>(2.);

        /* Init Delaunay triangulation. */
        auto d = Delaunay<T>{};

        const auto p0 = Node{ midx - 20 * dmax, midy - dmax, points[0].z };
        const auto p1 = Node{ midx, midy + 20 * dmax, points[0].z };
        const auto p2 = Node{ midx + 20 * dmax, midy - dmax, points[0].z };
        d.triangles.emplace_back(Triangle<T>{p0, p1, p2});

        for (auto const& pt : points)
        {
            std::vector<Edge<T>> edges;
            std::vector<Triangle<T>> tmps;
            for (auto const& tri : d.triangles)
            {
                /* Check if the point is inside the triangle circumcircle. */
                const auto dist = (tri.circle.x - pt.x) * (tri.circle.x - pt.x) +
                    (tri.circle.y - pt.y) * (tri.circle.y - pt.y);
                if ((dist - tri.circle.radius) <= eps)
                {
                    edges.push_back(tri.e0);
                    edges.push_back(tri.e1);
                    edges.push_back(tri.e2);
                }
                else
                {
                    tmps.push_back(tri);
                }
            }

            /* Delete duplicate edges. */
            std::vector<bool> remove(edges.size(), false);
            for (auto it1 = edges.begin(); it1 != edges.end(); ++it1)
            {
                for (auto it2 = edges.begin(); it2 != edges.end(); ++it2)
                {
                    if (it1 == it2)
                    {
                        continue;
                    }
                    if (*it1 == *it2)
                    {
                        remove[std::distance(edges.begin(), it1)] = true;
                        remove[std::distance(edges.begin(), it2)] = true;
                    }
                }
            }

            edges.erase(
                std::remove_if(edges.begin(), edges.end(),
                    [&](auto const& e) { return remove[&e - &edges[0]]; }),
                edges.end());

            /* Update triangulation. */
            for (auto const& e : edges)
            {
                tmps.push_back({ e.p0, e.p1, {pt.x, pt.y, pt.z} });
            }
            d.triangles = tmps;
        }

        /* Remove original super triangle. */
        d.triangles.erase(
            std::remove_if(d.triangles.begin(), d.triangles.end(),
                [&](auto const& tri)
                {
                    return ((tri.p0 == p0 || tri.p1 == p0 || tri.p2 == p0) ||
                        (tri.p0 == p1 || tri.p1 == p1 || tri.p2 == p1) ||
                        (tri.p0 == p2 || tri.p1 == p2 || tri.p2 == p2));
                }),
            d.triangles.end());

        /* Add edges. */
        for (auto const& tri : d.triangles)
        {
            d.edges.push_back(tri.e0);
            d.edges.push_back(tri.e1);
            d.edges.push_back(tri.e2);
        }
        return d;
    }

} /* namespace delaunay */

#pragma endregion

UExport::UExport()
{
    PrimaryComponentTick.bCanEverTick = true;
}

// Called every frame
void UExport::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

struct Face
{
    short x;
    short y;
    short z;
};

// Called when the game starts
void UExport::BeginPlay()
{
    Super::BeginPlay();

    ExportScene();
    //ExportNavMesh();
}

void UExport::ExportNavMesh()
{
    ARecastNavMesh* recastNavMesh = Cast<ARecastNavMesh>(FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld())->GetDefaultNavDataInstance());
    dtNavMesh* navMesh = recastNavMesh->GetRecastMesh();

    TArray<FVector> vertices;
    TArray<Face> faces;

    int indiceOffset = 0;

    for (int i = 0; i < navMesh->getMaxTiles(); ++i)
    {
        TArray<FNavPoly> polysInTile;

        if (!recastNavMesh->GetPolysInTile(i, polysInTile))
        {
            continue;
        }

        for (FNavPoly currentPoly : polysInTile)
        {
            FOccluderVertexArray verts;
            if (!recastNavMesh->GetPolyVerts(currentPoly.Ref, verts))
            {
                continue;
            }

            for (int j = 0; j < verts.Num(); ++j)
            {
                if (!vertices.Contains(verts[j]))
                    vertices.Add(verts[j]);
            }

            std::vector<delaunay::Point<float>> vertexVector;
            for (int j = 0; j < verts.Num(); ++j)
            {
                delaunay::Point<float> point(verts[j].X, verts[j].Y, verts[j].Z);
                vertexVector.push_back(point);
            }

            delaunay::Delaunay<float> triangles = delaunay::triangulate<float>(vertexVector);
            for (int j = 0; j < triangles.triangles.size(); ++j)
            {
                Face face;
                FVector vec0 = { triangles.triangles[j].p0.x, triangles.triangles[j].p0.y, triangles.triangles[j].p0.z };
                FVector vec1 = { triangles.triangles[j].p1.x, triangles.triangles[j].p1.y, triangles.triangles[j].p1.z };
                FVector vec2 = { triangles.triangles[j].p2.x, triangles.triangles[j].p2.y, triangles.triangles[j].p2.z };

                face.x = FindIndex(vec0, vertices) + 1;
                face.y = FindIndex(vec1, vertices) + 1;
                face.z = FindIndex(vec2, vertices) + 1;
                faces.Add(face);
            }
        }
    }

    std::string stdFilePath = TCHAR_TO_UTF8(*filePath);
    std::ofstream file(stdFilePath + "/NavMeshExport.obj");
    for (const FVector& vec : vertices)
    {
        file << "v " << vec.X << " " << vec.Z << " " << vec.Y << std::endl;
    }

    for (const Face& face : faces)
    {
        file << "f " << face.x << " " << face.y << " " << face.z << std::endl;
    }

    file << "#VerticesCount: " << vertices.Num() << std::endl;
    file << "#FaceCount: " << faces.Num() << std::endl;
    file << "#IndicdeOffset: " << indiceOffset << std::endl;

    file.close();
}

int UExport::FindIndex(const FVector& aKey, const TArray<FVector>& someVertices)
{
    for (int i = 0; i < someVertices.Num(); ++i)
    {
        if (someVertices[i] == aKey)
        {
            return i;
        }
    }
    return -1;
}

void UExport::ExportScene()
{
    TArray<AActor*> actorsFound;
    TSubclassOf<AActor> classToFind = AActor::StaticClass();
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), classToFind, actorsFound);

    nlohmann::json jsonFile;
    jsonFile["fileVersion"] = "2.0";
    nlohmann::json& root = jsonFile["root"];
    root["name"] = "UnrealScene";
    root["components"] = std::vector<nlohmann::json>();
    for (size_t i = 0; i < actorsFound.Num(); i++)
    {
        root["children"][i] = CreateEntity(*actorsFound[i]);
    }

    const std::string outPath = std::string(TCHAR_TO_UTF8(*filePath)) + "/Export.fab";
    const FString outPathFString = outPath.c_str();
    UE_LOG(LogExporter, Display, TEXT("Saved export to \"%s\""), *outPathFString);
    std::ofstream file(outPath);
    file << std::setw(4) << jsonFile;
    file.close();
}

nlohmann::json UExport::CreateComponents(const AActor& aActor) 
{
    std::vector<nlohmann::json> components;

    //Transforms
    {
        const FTransform& src = aActor.GetTransform();
        nlohmann::json dst;
        dst["type"] = "TransformData";
        dst["pos"] = CreateFVectorJson(ToExportPos(src.GetLocation()));
        dst["rot"] = CreateFVectorJson(ToExportRot(src.GetRotation().Euler()));
        dst["scale"] = CreateFVectorJson(ToExportPos(src.GetScale3D()));
        components.push_back(dst);
    }

    //Pointlights
    TArray<UPointLightComponent*> pointLights;
    aActor.GetComponents<UPointLightComponent>(pointLights);
    for (int32 i = 0; i < pointLights.Num(); i++)
    {
        UPointLightComponent& src = *pointLights[i];
    	nlohmann::json dst;
        dst["type"] = "PointLightData";
        dst["color"] = CreateColorJson(src.GetLightColor());
        dst["intensity"] = src.Intensity;
        dst["range"] = src.SourceRadius;
        components.push_back(dst);
    }

    //Spotlights
    TArray<USpotLightComponent*> spotLights;
    aActor.GetComponents<USpotLightComponent>(spotLights);
    for (int32 i = 0; i < spotLights.Num(); i++)
    {
        USpotLightComponent& src = *spotLights[i];
        nlohmann::json dst;
        dst["type"] = "SpotLightData";
        dst["color"] = CreateColorJson(src.GetLightColor());
        dst["intensity"] = src.Intensity;
        dst["range"] = src.SourceRadius;
        dst["innerRadius"] = src.InnerConeAngle;
        dst["outerRadius"] = src.OuterConeAngle;
        components.push_back(dst);
    }

    //DirectionalLights
    TArray<UDirectionalLightComponent*> directionalLights;
    aActor.GetComponents<UDirectionalLightComponent>(directionalLights);
    for (int32 i = 0; i < directionalLights.Num(); i++)
    {
        UDirectionalLightComponent& src = *directionalLights[i];
        nlohmann::json dst;
        dst["type"] = "DirectionalLightData";
        dst["color"] = CreateColorJson(src.GetLightColor());
        dst["intensity"] = src.Intensity;
        components.push_back(dst);
    }

    //MeshRenderers
    TArray<UStaticMeshComponent*> staticMeshes;
    aActor.GetComponents<UStaticMeshComponent>(staticMeshes);
    for (int32 i = 0; i < staticMeshes.Num(); i++)
    {
        UStaticMeshComponent& src = *staticMeshes[i];
        nlohmann::json dst;
        dst["type"] = "MeshRendererData";
        const FString pathPrefix = "Assets";
        const FString badPathPrefix = "Content";
        const FString invalidPath = "???";
        FString path = src.GetStaticMesh()->AssetImportData->GetFirstFilename();
        if (FPaths::MakePathRelativeTo(path, ToCStr(FPaths::ProjectDir())))
        {
            UE_LOG(LogExporter, Display, TEXT("WEEEE: %s"), *path.Left(badPathPrefix.Len()))
            if (path.Left(badPathPrefix.Len()) == badPathPrefix)
            {
                path = pathPrefix + path.RightChop(badPathPrefix.Len());
            }
        	else
            {
                UE_LOG(LogExporter, Warning, TEXT("Bad model path! Failed to replace root directory. Skipping \"%s\""), *path)
        		path = invalidPath;
            }
        }
    	else
        {
	        UE_LOG(LogExporter, Warning, TEXT("Bad model path! Failed to make path relative. Skipping \"%s\""), *path)
	        path = invalidPath;
        }
    	dst["modelPath"] = TCHAR_TO_UTF8(ToCStr(path));
        components.push_back(dst);
    }

    //Cameras
    TArray<UCameraComponent*> cameras;
    aActor.GetComponents<UCameraComponent>(cameras);
    for (int32 i = 0; i < cameras.Num(); i++)
    {
        UCameraComponent& src = *cameras[i];
        nlohmann::json dst;
        dst["type"] = "CameraData";
        dst["fov"] = src.FieldOfView;
        dst["nearPlane"] = nearPlane;
        dst["farPlane"] = farPlane;
        components.push_back(dst);
    }
    
    return components;
}

nlohmann::json UExport::CreateEntity(const AActor& aActor) 
{
	nlohmann::json entity;
    entity["name"] = TCHAR_TO_UTF8(ToCStr(aActor.GetName()));

	entity["components"] = CreateComponents(aActor);
    
	for (size_t i = 0; i < aActor.Children.Num(); i++)
	{
        entity["children"][i] = CreateEntity(*aActor.Children[i]);
	}

	return entity;
}

nlohmann::json UExport::CreateFVectorJson(const FVector& aVector)
{
	return {
        {"x", aVector.X},
        {"y", aVector.Y},
        {"z", aVector.Z}
    };
}
nlohmann::json UExport::CreateColorJson(const FLinearColor& aColor)
{
	return {
        {"r", aColor.R},
        {"g", aColor.G},
        {"b", aColor.B},
        {"a", aColor.A}
    };
}

FVector UExport::ToExportPos(const FVector& aPos)
{
    FVector result;
    result.X = aPos.Y;
    result.Y = aPos.Z;
    result.Z = aPos.X;
    return result;
}

FVector UExport::ToExportRot(const FVector& aRot)
{
    FVector result;
    result.X = -aRot.Y;
    result.Y = aRot.Z;
    result.Z = -aRot.X;
    return result;
}
