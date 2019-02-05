%initialize as empty
hit_policies = [];

while true
    hit_policies = new_hit_policy(hit_policies);
    
    send_hit_policies(hit_policies(:, 1:8), hit_policies(:, 9:17));
end