// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "json.hpp"
#include "Components/PointLightComponent.h"
#include "Export.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogExporter, Log, All);

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class MYPROJECT2_API UExport : public UActorComponent
{
	GENERATED_BODY()

public:
	// Sets default values for this component's properties
	UExport();

	UPROPERTY(EditAnywhere) FString filePath;
	UPROPERTY(EditAnywhere) bool shouldAutoFixLights = false;
	UPROPERTY(EditAnywhere) float nearPlane = 0.1f;
	UPROPERTY(EditAnywhere) float farPlane = 1000.0f;

	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

protected:
	// Called when the game starts
	virtual void BeginPlay() override;

private:
	void ExportNavMesh(const std::string& aOutPath);
	int FindIndex(const FVector& aKey, const TArray<FVector>& someVertices);

	void ExportScene(const std::string& aOutPath);
	nlohmann::json CreateEntity(const AActor& aActor);
	nlohmann::json CreateComponents(const AActor& aActor);
	void CheckLight(UPointLightComponent& aLight);

	nlohmann::json CreateFVectorJson(const FVector& aVector);
	nlohmann::json CreateFQuatJson(const FQuat& aQuat);
	nlohmann::json CreateColorJson(const FLinearColor& aColor);

	static FVector ToExportPos(const FVector& aPos);
	static FQuat ToExportRot(const FQuat& aRot);
	static FVector ToExportScale(const FVector& aScale);
};
