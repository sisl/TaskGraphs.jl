let
    n1 = ROBOT_AT(1,2)
    n2 = GO(-1,-1,3)
    n = align_with_predecessor(n2,n1)
    @test n.r == n1.r
    @test n.x1 == n1.x
end
let
    n1 = DEPOSIT(1,2,3)
    n2 = GO(-1,3,4)
    n = align_with_predecessor(n2,n1)
    @test n.r == n1.r
    @test n.x1 == n1.x
end
let
    n1 = GO(1,2,4)
    n2 = COLLECT(-1,3,4)
    n = align_with_predecessor(n2,n1)
    @test n.r == n1.r
    @test n.x == n1.x2
end
let
    n1 = OBJECT_AT(3,4)
    n2 = COLLECT(-1,3,4)
    n = align_with_predecessor(n2,n1)
    @test n.o == n1.o
    @test n.x == n1.x[1]
end
let
    n1 = COLLECT(1,2,3)
    n2 = CARRY(-1,2,3,4)
    n = align_with_predecessor(n2,n1)
    @test n.o == n1.o
    @test n.x1 == n1.x
    @test n.r == n1.r
end
let
    n1 = CARRY(1,2,3,4)
    n2 = DEPOSIT(-1,2,4)
    n = align_with_predecessor(n2,n1)
    @test n.o == n1.o
    @test n.x == n1.x2
    @test n.r == n1.r
end
