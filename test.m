x = -50:0.01:50;

y = 100./(1+x.^2).^2;
plot(x,y)

%%
ids = getClusterIds(allPredVectors, clusteringModel);
y = zeros(1,10);
for id = 1:10
    y(id) = sum(ids == id);
end
figure
bar(1:10, y)