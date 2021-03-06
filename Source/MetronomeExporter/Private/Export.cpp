#include "Export.h"
#include "Detour/DetourNavMesh.h"
#include "NavMesh/RecastNavMesh.h"
#include "NavigationSystem.h"
#include "NavigationSystem\Public\NavMesh\PImplRecastNavMesh.h"
#include "Components/LightComponent.h"
#include "Components/PointLightComponent.h"
#include "Components/SpotLightComponent.h"
#include "Components/DirectionalLightComponent.h"
#include "Components/BoxComponent.h"
#include "Components/SphereComponent.h"
#include "Camera/CameraComponent.h"
#include "EditorFramework/AssetImportData.h"
#include "Kismet/GameplayStatics.h"
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#undef min
#undef max

DEFINE_LOG_CATEGORY(LogExporter);

#pragma region Triangulation

#include <algorithm>
#include <vector>
namespace delaunay
{
	constexpr double eps = 1e-4;

	template <typename T>
	struct Point
	{
		T x, y, z;

		Point() : x{ 0 }, y{ 0 }, z{ 0 } {}
		Point(T _x, T _y, T _z) : x{ _x }, y{ _y }, z{ _z } {}

		template <typename U>
		Point(U _x, U _y, U _z) : x{ static_cast<T>(_x) }, y{ static_cast<T>(_y) }, z{ static_cast<T>(_z) }
		{
		}

		friend std::ostream& operator<<(std::ostream& os, const Point<T>& p)
		{
			os << "x=" << p.x << "  y=" << p.y;
			return os;
		}

		bool operator==(const Point<T>& other) const
		{
			return (other.x == x && other.y == y);
		}

		bool operator!=(const Point<T>& other) const { return !operator==(other); }
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
				[&](auto const& tri) {
					return (
						(tri.p0 == p0 || tri.p1 == p0 || tri.p2 == p0) ||
						(tri.p0 == p1 || tri.p1 == p1 || tri.p2 == p1) ||
						(tri.p0 == p2 || tri.p1 == p2 || tri.p2 == p2)
						);
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
}

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

	UE_LOG(LogExporter, Display, TEXT("New export started!"));

	const std::string stdSceneExportName = TCHAR_TO_UTF8(*sceneExportName);
	const std::string stdSceneExportPath = TCHAR_TO_UTF8(*sceneExportPath);
	ExportScene(stdSceneExportPath + "/" + stdSceneExportName + ".fab");
	ExportNavMesh(stdSceneExportPath + "/" + stdSceneExportName + "Nav.obj");

	UE_LOG(LogExporter, Display, TEXT("Saved export to \"%s\""), *sceneExportPath);
}

void UExport::ExportNavMesh(const std::string& aOutPath)
{
	ARecastNavMesh* recastNavMesh = Cast<ARecastNavMesh>(FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld())->GetDefaultNavDataInstance());
	if (recastNavMesh == nullptr)
	{
		UE_LOG(LogExporter, Warning, TEXT("No Navmesh detected, Skipping..."))
			return;
	}
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
				face.y = FindIndex(vec2, vertices) + 1;
				face.z = FindIndex(vec1, vertices) + 1;
				faces.Add(face);
			}
		}
	}

	std::ofstream file(aOutPath);
	for (const FVector& vec : vertices)
	{
		FVector newVec = ToExportFVector(vec);
		file << "v " << newVec.X << " " << newVec.Y << " " << newVec.Z << std::endl;
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

void UExport::ExportScene(const std::string& aOutPath)
{
	context = ExportContext();

	TArray<AActor*> actorsFound;
	TSubclassOf<AActor> classToFind = AActor::StaticClass();
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), classToFind, actorsFound);

	Folder root;
	for (size_t i = 0; i < actorsFound.Num(); i++)
	{
		AActor* actor = actorsFound[i];
		//const FString folderPath = actor->GetFolderPath().ToString();
		const FString folderPath = ("#" + actor->GetFolderPath().ToString() + "#").LeftChop(1).RightChop(1); //i have no clue why but for some reason the wide string in here doesn't play nice without this

		Folder* folder = &root;
		if (folderPath != "None") {
			TArray<FString> folderNames;
			folderPath.ParseIntoArray(folderNames, TEXT("/"), true);
			for (FString& folderName : folderNames)
			{
				const std::string folderStr = TCHAR_TO_UTF8(ToCStr(folderName));

				const auto it = folder->mySubFolders.find(folderStr);
				if (it != folder->mySubFolders.end())
				{
					folder = &it->second;
				} else
				{
					folder = &folder->mySubFolders.insert({ folderStr, {} }).first->second;
				}
			}
		}
		folder->myActors.Push(actor);
	}

	nlohmann::json json;
	json["fileVersion"] = "3.1";
	json["root"] = CreateFolderEntity("UnrealScene", root);;
	WriteJsonToFile(aOutPath, json);
}

nlohmann::json UExport::CreateComponents(const AActor& aActor)
{
	nlohmann::json components;

	//NameTag
	components.push_back(CreateComponentJson("NameTag", CreateNameTagJson(TCHAR_TO_UTF8(ToCStr(aActor.GetActorLabel())))));

	//Parent
	components.push_back(CreateComponentJson("Parent", CreateParentJson(aActor.Children)));

	//Transform
	components.push_back(CreateComponentJson("Transform", CreateTransformJson(aActor.GetTransform())));

	//Pointlights
	ForeachComponent<UPointLightComponent>(aActor, [&](UPointLightComponent& aSrc){
		CheckLight(aSrc);
		components.push_back(CreateComponentJson("PointLight", CreatePointLightJson(aSrc)));
	});

	//Spotlights
	ForeachComponent<USpotLightComponent>(aActor, [&](USpotLightComponent& aSrc) {
		CheckLight(aSrc);
		components.push_back(CreateComponentJson("SpotLight", CreateSpotLightJson(aSrc)));
	});

	//DirectionalLights
	ForeachComponent<UDirectionalLightComponent>(aActor, [&](UDirectionalLightComponent& aSrc) {
		components.push_back(CreateComponentJson("DirectionalLight", CreateDirectionalLightJson(aSrc)));
	});

	//MeshRenderers
	ForeachComponent<UStaticMeshComponent>(aActor, [&](UStaticMeshComponent& aSrc) {
		UStaticMesh* staticMesh = aSrc.GetStaticMesh();
		if (staticMesh == nullptr) return;

		FString modelPath = staticMesh->AssetImportData->GetFirstFilename();
		if (modelPath == "C:/Program Files/Epic Games/UE_4.27/Engine/Content/EditorMeshes/MatineeCam_SM.FBX") return;

		nlohmann::json params;

		//process path
		switch (ResolvePath(modelPath, "Content", "Assets"))
		{
		case ResolvePathResult::Success: {
			const FString rawExt = ".fbx";
			const FString exportExt = ".wardh";
			if (modelPath.EndsWith(rawExt))
			{
				modelPath = modelPath.Replace(&rawExt[0], &exportExt[0]); //slightly unreliable but it'll do...
			}
			else
			{
				UE_LOG(LogExporter, Warning, TEXT("Bad model path! Failed to find fbx extension. Exporting with raw extension..."), *modelPath)
			}
			break;
		}
		case ResolvePathResult::MakeRelativeFailed:
			UE_LOG(LogExporter, Error, TEXT("Bad model path! Failed to make path relative. Skipping \"%s\""), *modelPath)
				modelPath = modelFallbackPath;
			break;
		case ResolvePathResult::PrefixFailed:
			UE_LOG(LogExporter, Error, TEXT("Bad model path! Failed to replace root directory. Skipping \"%s\""), *modelPath)
				modelPath = modelFallbackPath;
			break;
		}
		params["modelPath"] = TCHAR_TO_UTF8(ToCStr(modelPath));

		//process materials
		for (const UMaterialInterface* material : aSrc.GetMaterials())
		{
			FString materialPath;
			if (material != nullptr) {
				FString name = material->GetName();
				if (name != "WorldGridMaterial")
				{
					EnsureFolder("Materials");
					materialPath = "Assets/Materials/" + name + ".mat";
					FString materialExportPath = "Materials/" + name + ".mat";
					EnsureMaterial(materialPath);
				}
				else
				{
					materialPath = materialFallbackPath;
				}
			}
			else
			{
				materialPath = materialFallbackPath;
			}
			params["materials"].push_back(TCHAR_TO_UTF8(ToCStr(materialPath)));
		}

		components.push_back(CreateComponentJson("MeshRenderer", params));
	});

	//Cameras
	ForeachComponent<UCameraComponent>(aActor, [&](UCameraComponent& aSrc) {
		nlohmann::json params;
		params["fov"] = aSrc.FieldOfView;
		params["nearPlane"] = nearPlane;
		params["farPlane"] = farPlane;
		components.push_back(CreateComponentJson("Camera", params));
	});

	//Box Collider
	ForeachComponent<UBoxComponent>(aActor, [&](UBoxComponent& aSrc) {
		nlohmann::json params;
		params["size"] = CreateFVectorJson(ToExportFVector(aSrc.GetUnscaledBoxExtent() * 2));
		components.push_back(CreateComponentJson("BoxCollider", params));

	});

	//Box Collider
	ForeachComponent<USphereComponent>(aActor, [&](USphereComponent& aSrc) {
		nlohmann::json params;
		params["radius"] = aSrc.GetUnscaledSphereRadius();
		components.push_back(CreateComponentJson("SphereCollider", params));

	});

	return components;
}

nlohmann::json UExport::CreateComponentJson(const std::string& aType, const nlohmann::json& aParams)
{
	nlohmann::json result;

	result["type"] = aType;
	result["params"] = aParams;

	return result;
}

void UExport::CheckLight(UPointLightComponent& aLight)
{
	constexpr float correctFalloffExponent = 2;
	if (shouldAutoFixLights)
	{
		aLight.bUseInverseSquaredFalloff = false;
		aLight.LightFalloffExponent = correctFalloffExponent;
	}

	const FString name = aLight.GetName();
	if (aLight.bUseInverseSquaredFalloff)
	{
		UE_LOG(LogExporter, Warning, TEXT("Bad light (%s)! Inverse square falloff is not allowed. Turn it off!"), *name)
	}
	if (aLight.LightFalloffExponent != correctFalloffExponent)
	{
		const wchar_t* falloffExponentStr = UTF8_TO_TCHAR(std::to_string(aLight.LightFalloffExponent).c_str());
		const wchar_t* correctFalloffExponentStr = UTF8_TO_TCHAR(std::to_string(correctFalloffExponent).c_str());
		UE_LOG(LogExporter, Warning, TEXT("Bad light (%s)! falloff exponent is %s it needs to be %s"), *name, falloffExponentStr, correctFalloffExponentStr)
	}
}

void UExport::WriteJsonToFile(const std::string& aPath, const nlohmann::json& aJson) const
{
	std::ofstream stream(aPath);
	if (!shouldMakeCompactJson)
	{
		stream << std::setw(4);
	}
	stream << aJson;
	stream.close();
}

nlohmann::json UExport::CreateNameTagJson(const std::string& aName)
{
	nlohmann::json result;

	result["name"] = aName;

	return result;
}

nlohmann::json UExport::CreateParentJson(const TArray<AActor*>& someChildren)
{
	nlohmann::json result;

	for (size_t i = 0; i < someChildren.Num(); i++)
	{
		result["children"][i] = CreateEntity(*someChildren[i]);
	}

	return result;
}

nlohmann::json UExport::CreateTransformJson(const FTransform& aSrc)
{
	nlohmann::json result;

	result["pos"] = CreateFVectorJson(ToExportFVector(aSrc.GetLocation() * 0.01f));
	//result["rot"] = CreateFVectorJson(ToExportPos(aSrc.GetRotation().Euler()));
	//result["rot"] = CreateFQuatJson(ToExportRot(aSrc.GetRotation()));
	result["scale"] = CreateFVectorJson(ToExportFVector(aSrc.GetScale3D()).GetAbs());



	{ //test
	  //STAGE 1: get matrix data
		FMatrix mat = FRotationMatrix::Make(aSrc.GetRotation());
		const float m11 = mat.M[0][0];
		const float m12 = mat.M[0][1];
		const float m13 = mat.M[0][2];

		const float m22 = mat.M[1][1];
		const float m23 = mat.M[1][2];

		const float m32 = mat.M[2][1];
		const float m33 = mat.M[2][2];

		//STAGE 2: use matrix to make euler in the xyz order
		//https://github.com/mrdoob/three.js/blob/8ff5d832eedfd7bc698301febb60920173770899/src/math/Euler.js#L104
		FVector xyzEuler;
		xyzEuler.Y = asin(FMath::Clamp(m13, -1.0f, 1.0f));

		if (FMath::Abs(m13) < 0.9999999f) 
		{
			xyzEuler.X = FMath::Atan2(-m23, m33);
			xyzEuler.Z = FMath::Atan2(-m12, m11);
		}
		else 
		{
			xyzEuler.X = FMath::Atan2(m32, m22);
			xyzEuler.Z = 0;
		}

		constexpr float radToDeg = 180 / 3.14159265359f;
		result["rot"] = CreateFVectorJson(ToExportFVector(-xyzEuler * radToDeg));
	}
	return result;
}

UExport::ResolvePathResult UExport::ResolvePath(FString& aPath, const FString& aIncorrectPathPrefix, const FString& aCorrectPathPrefix)
{
	if (!FPaths::MakePathRelativeTo(aPath, ToCStr(FPaths::ProjectDir()))) return ResolvePathResult::MakeRelativeFailed;

	if (aPath.Left(aIncorrectPathPrefix.Len()) != aIncorrectPathPrefix) return ResolvePathResult::PrefixFailed;

	aPath = aCorrectPathPrefix + aPath.RightChop(aIncorrectPathPrefix.Len());
	return ResolvePathResult::Success;
}

void UExport::ExportMaterial(const FString& aPath)
{
	nlohmann::json json;

	WriteJsonToFile(std::string(TCHAR_TO_UTF8(ToCStr(aPath))), json);
	UE_LOG(LogExporter, Display, TEXT("Material exported. \"%s\""), *aPath)
}

void UExport::EnsureMaterial(const FString& aPath)
{
	if (std::find(context.myMaterialCache.begin(), context.myMaterialCache.end(), aPath) != context.myMaterialCache.end()) return;
	ExportMaterial(aPath);
	context.myMaterialCache.push_back(aPath);
};

void UExport::EnsureFolder(const FString& aPath)
{
	CreateDirectory(ToCStr(aPath), NULL);
}

nlohmann::json UExport::CreateLightJson(const ULightComponent& aSrc)
{
	nlohmann::json result;

	result["color"] = CreateColorJson(aSrc.GetLightColor());
	result["intensity"] = aSrc.Intensity;

	return result;
}

nlohmann::json UExport::CreatePointLightJson(const UPointLightComponent& aSrc)
{
	nlohmann::json result = CreateLightJson(aSrc);

	result["range"] = aSrc.AttenuationRadius * 0.01f;

	return result;
}

nlohmann::json UExport::CreateSpotLightJson(const USpotLightComponent& aSrc)
{
	nlohmann::json result = CreatePointLightJson(aSrc);

	result["innerRadius"] = aSrc.InnerConeAngle;
	result["outerRadius"] = aSrc.OuterConeAngle;

	return result;
}

nlohmann::json UExport::CreateDirectionalLightJson(const UDirectionalLightComponent& aSrc)
{
	nlohmann::json result = CreateLightJson(aSrc);

	//no actions needed

	return result;
}

nlohmann::json UExport::CreateEntity(const AActor& aActor)
{
	nlohmann::json entity;

	entity["components"] = CreateComponents(aActor);

	return entity;
}

nlohmann::json UExport::CreateFolderEntity(const std::string& aName, const Folder& aFolder) {
	nlohmann::json entity;
	nlohmann::json& components = entity["components"];

	components.push_back(CreateComponentJson("NameTag", CreateNameTagJson(aName + " [FOLDER]")));
	nlohmann::json parent = CreateComponentJson("Parent", CreateParentJson(aFolder.myActors));
	nlohmann::json children; //we use this to force the folders to the top of the hierarchy
	for (std::pair<const std::string, Folder> pair : aFolder.mySubFolders)
	{
		children.push_back(CreateFolderEntity(pair.first, pair.second));
	}
	for (nlohmann::json& child: parent["params"]["children"])
	{
		children.push_back(child);
	}
	parent["params"]["children"] = children;
	components.push_back(parent);

	return entity;
}

nlohmann::json UExport::CreateFVectorJson(const FVector& aSrc)
{
	return {
		{"x", aSrc.X},
		{"y", aSrc.Y},
		{"z", aSrc.Z}
	};
}
nlohmann::json UExport::CreateFQuatJson(const FQuat& aSrc)
{
	return {
		{"x", aSrc.X},
		{"y", aSrc.Y},
		{"z", aSrc.Z},
		{"w", aSrc.W}
	};
}
nlohmann::json UExport::CreateColorJson(const FLinearColor& aSrc)
{
	return {
		{"r", aSrc.R},
		{"g", aSrc.G},
		{"b", aSrc.B},
		{"a", aSrc.A}
	};
}

FVector UExport::ToExportFVector(const FVector& aSrc)
{
	FVector result;

	result.X = aSrc.Y;
	result.Y = aSrc.Z;
	result.Z = aSrc.X;

	return result;
}
FQuat UExport::ToExportFQuat(const FQuat& aSrc)
{
	//STAGE 1: get matrix data
	FMatrix mat = FRotationMatrix::Make(aSrc);
	const float m11 = mat.M[0][0];
	const float m12 = mat.M[0][1];
	const float m13 = mat.M[0][2];

	const float m22 = mat.M[1][1];
	const float m23 = mat.M[1][2];

	const float m32 = mat.M[2][1];
	const float m33 = mat.M[2][2];

	//STAGE 2: use matrix to make euler in the xyz order
	//https://github.com/mrdoob/three.js/blob/8ff5d832eedfd7bc698301febb60920173770899/src/math/Euler.js#L104
	FVector xyzEuler;
	xyzEuler.Y = asin(FMath::Clamp(m13, -1.0f, 1.0f));

	if (FMath::Abs(m13) < 0.9999999f) 
	{
		xyzEuler.X = FMath::Atan2(-m23, m33);
		xyzEuler.Z = FMath::Atan2(-m12, m11);
	}
	else 
	{
		xyzEuler.X = FMath::Atan2(m32, m22);
		xyzEuler.Z = 0;
	}


	//STAGE 3: make quaternion in xyz order using new xyz euler
	//https://github.com/mrdoob/three.js/blob/61a4d5c90034e904d77f2787ee11dc512e51968d/src/math/Quaternion.js#L206
	const float c1 = cos(xyzEuler.X / 2);
	const float c2 = cos(xyzEuler.Y / 2);
	const float c3 = cos(xyzEuler.Z / 2);

	const float s1 = sin(xyzEuler.X / 2);
	const float s2 = sin(xyzEuler.Y / 2);
	const float s3 = sin(xyzEuler.Z / 2);

	FQuat xyzQuat;
	xyzQuat.X = s1 * c2 * c3 + c1 * s2 * s3;
	xyzQuat.Y = c1 * s2 * c3 - s1 * c2 * s3;
	xyzQuat.Z = c1 * c2 * s3 + s1 * s2 * c3;
	xyzQuat.W = c1 * c2 * c3 - s1 * s2 * s3;

	//STAGE 4: swizzle xyzQuat into the right component layout for metronome
	FQuat result;
	result.X = xyzQuat.Y;
	result.Y = xyzQuat.Z;
	result.Z = xyzQuat.X;
	result.W = -xyzQuat.W;

	//result.X = aRot.Y;
	//result.Y = aRot.Z;
	//result.Z = aRot.X;
	//result.W = aRot.W;
	//result.Normalize(); //we shouldn't need this but better safe than sorry

	return result;
}
