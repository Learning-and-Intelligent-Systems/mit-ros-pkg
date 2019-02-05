function draw_table()

table_length = 2.76;
table_width = 1.525;

P = [0,0,0; table_length,0,0; table_length,table_width,0; 0,table_width,0; 0,0,0];
P2 = [0,table_width/2,0; table_length,table_width/2,0];
plot3(P(:,1), P(:,2), P(:,3), 'k-', 'LineWidth', 2);
hold on
plot3(P2(:,1), P2(:,2), P2(:,3), 'k-', 'LineWidth', 2);
hold off
