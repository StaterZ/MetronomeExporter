// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "json.hpp"
#include "Components/DirectionalLightComponent.h"
#include "Components/PointLightComponent.h"
#include "Components/SpotLightComponent.h"
#include "Export.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogExporter, Log, All);

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class METRONOMEEXPORTER_API UExport : public UActorComponent
{
	GENERATED_BODY()

public:
	// Sets default values for this component's properties
	UExport();

	UPROPERTY(EditAnywhere) FString sceneExportPath;
	UPROPERTY(EditAnywhere) FString sceneExportName = "Export";
	UPROPERTY(EditAnywhere) bool shouldMakeCompactJson = true;
	UPROPERTY(EditAnywhere) bool shouldAutoFixLights = false;
	UPROPERTY(EditAnywhere) float nearPlane = 0.1f;
	UPROPERTY(EditAnywhere) float farPlane = 100000.0f;
	UPROPERTY(EditAnywhere) FString modelFallbackPath = "???";
	UPROPERTY(EditAnywhere) FString materialFallbackPath = "???";

	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

private:
	struct ExportContext
	{
		std::vector<FString> myMaterialCache;
	};

	void ExportNavMesh(const std::string& aOutPath);
	int FindIndex(const FVector& aKey, const TArray<FVector>& someVertices);

	void ExportScene(const std::string& aOutPath);
	nlohmann::json CreateEntity(const AActor& aActor);
	nlohmann::json CreateComponents(const AActor& aActor);
	static nlohmann::json CreateComponentJson(const std::string& aType, const nlohmann::json& aParams);

	template<typename T>
	static void ForeachComponent(const AActor& aActor, const std::function<void(T&)>& aFunc);

	void CheckLight(UPointLightComponent& aLight);
	void WriteJsonToFile(const std::string& aPath, const nlohmann::json& aJson) const;

	enum class ResolvePathResult {
		Success,
		MakeRelativeFailed,
		PrefixFailed
	};
	ResolvePathResult ResolvePath(FString& aPath, const FString& aIncorrectPathPrefix, const FString& aCorrectPathPrefix);

	void ExportMaterial(const FString& aPath);
	void EnsureMaterial(const FString& aPath);
	void EnsureFolder(const FString& aPath);

	nlohmann::json CreateNameTagJson(const std::string& aName);
	nlohmann::json CreateParentJson(const TArray<AActor*>& someChildren);
	nlohmann::json CreateTransformJson(const FTransform& aSrc);
	nlohmann::json CreateLightJson(const ULightComponent& aSrc);
	nlohmann::json CreatePointLightJson(const UPointLightComponent& aSrc);
	nlohmann::json CreateSpotLightJson(const USpotLightComponent& aSrc);
	nlohmann::json CreateDirectionalLightJson(const UDirectionalLightComponent& aSrc);

	nlohmann::json CreateFVectorJson(const FVector& aSrc);
	nlohmann::json CreateFQuatJson(const FQuat& aSrc);
	nlohmann::json CreateColorJson(const FLinearColor& aSrc);

	static FVector ToExportFVector(const FVector& aSrc);
	static FQuat ToExportFQuat(const FQuat& aSrc);

	ExportContext context;
};

template <typename T>
void UExport::ForeachComponent(const AActor& aActor, const std::function<void(T&)>& aFunc)
{
	TArray<T*> componentTs;
	aActor.GetComponents<T>(componentTs);
	for (int32 i = 0; i < componentTs.Num(); i++)
	{
		aFunc(*componentTs[i]);
	}
}
